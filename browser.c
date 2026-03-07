#include <assert.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#define TINY_LA_IMPLEMENTATION
#include <emscripten.h>
#include <math.h>
#include <stdio.h>

typedef struct Color {
  unsigned char r;
  unsigned char g;
  unsigned char b;
  unsigned char a;
} Color;

// =====================================
//
// Arena
//
// =====================================

typedef struct {
  char *base;
  size_t offset;
  size_t capacity;
} Arena;

Arena arena_create(size_t capacity) {
  Arena a = {0};
  a.capacity = capacity;
  a.base = malloc(capacity);
  return a;
}

void arena_destroy(Arena *a) { free(a->base); }

void *arena_alloc(Arena *a, size_t amount) {
  size_t aligned = (amount + 7) & ~7;
  if (a->offset + aligned > a->capacity) {
    return NULL;
  }
  char *addr = a->base + a->offset;
  a->offset += aligned;
  return addr;
}

size_t arena_save(Arena *a) { return a->offset; }

void arena_restore(Arena *a, size_t save_point) {
  assert(a->offset >= save_point);
  a->offset = save_point;
}

// =====================================
//
// Constants
//
// =====================================

float R = 8.314;
float M_air = 0.029; // mol/kg

// =====================================
//
// Physics
//
// =====================================

float gas_density(float R_g, float P, float T) { return P / (R_g * T); }
float sound_speed(float rho, float gamma, float p) {
  return sqrtf(gamma * p / rho);
};

// =====================================
//
// Geometry
//
// =====================================

typedef struct Vertex2 {
  float x;
  float y;
} Vertex2;

typedef struct {
  Vertex2 *vertices;
  size_t vertex_count;
} Polygon;

typedef struct {
  Vertex2 *points;
  size_t point_count;
} PointCloud;

typedef struct {
  int v[3];
} Triangle;

typedef struct {
  int a;
  int b;
} Edge;

typedef struct {
  float x;
  float y;
  float r;
} Circle;

typedef struct {
  Vertex2 *points;
  size_t num_points;
  Triangle *tris;
  size_t num_tris;
} TriangularMesh;

typedef struct HalfEdge {
  int origin;
  int face;
  int next;
  int twin;
} HalfEdge;

typedef struct {
  HalfEdge *half_edges;
  size_t num_half_edges;

  int *face_edges; // Fist half-edge for each triangle
  int *vert_edge;  // On outgoing half-edge of each vertex

} HalfEdgeMesh;

Polygon create_polygon(Arena *a, size_t capacity) {
  Polygon poly = {0};
  poly.vertices = arena_alloc(a, capacity * sizeof(Vertex2));
  return poly;
}

PointCloud create_point_cloud(Arena *a, size_t capacity) {
  PointCloud cloud = {0};
  cloud.points = arena_alloc(a, capacity * sizeof(Vertex2));
  return cloud;
}

TriangularMesh create_triangular_mesh(Arena *a, size_t capacity) {
  TriangularMesh mesh = {0};
  mesh.tris = arena_alloc(a, capacity * sizeof(Triangle));
  return mesh;
}

HalfEdgeMesh create_half_edge_mesh(Arena *a, size_t num_points,
                                   size_t num_tris) {
  HalfEdgeMesh mesh = {0};
  mesh.half_edges = arena_alloc(a, 3 * num_tris * sizeof(HalfEdge));
  memset(mesh.half_edges, 0, 3 * num_tris * sizeof(HalfEdge));
  mesh.face_edges = arena_alloc(a, num_tris * sizeof(int));
  memset(mesh.face_edges, 0, num_tris * sizeof(int));
  mesh.vert_edge = arena_alloc(a, num_points * sizeof(int));
  memset(mesh.vert_edge, -1, num_points * sizeof(int));
  return mesh;
}

float vec(Vertex2 u, Vertex2 v) {
  // The vector product formula for u x v:
  // (ux*i + uy*j) x (vx*i + vy*j) = (ux * vy - uy * vx) * k
  return u.x * v.y - u.y * v.x;
}

float ccw(Vertex2 a, Vertex2 b, Vertex2 c) {
  // The vector product of a-b and b-c must point
  // out from the screen
  Vertex2 ab = {.x = b.x - a.x, .y = b.y - a.y};
  Vertex2 bc = {.x = c.x - b.x, .y = c.y - b.y};
  return vec(ab, bc) > 0;
}

double det3(                      //
    double a, double b, double c, //
    double d, double e, double f, //
    double g, double h, double i) {
  return a * e * i + b * f * g + c * d * h -
         (g * e * c + h * f * a + i * d * b);
}

float triangle_area(Vertex2 a, Vertex2 b, Vertex2 c) {
  Vertex2 ab = {.x = b.x - a.x, .y = b.y - a.y};
  Vertex2 bc = {.x = c.x - b.x, .y = c.y - b.y};
  return 0.5 * fabsf(vec(ab, bc));
}

float segment_length(Vertex2 a, Vertex2 b) {
  float dx = a.x - b.x;
  float dy = a.y - b.y;
  return sqrtf(dx * dx + dy * dy);
}

void get_vertices_extrema(Vertex2 *vertices, size_t vertex_count, float *xmin,
                          float *xmax, float *ymin, float *ymax) {
  *xmin = vertices[0].x;
  *xmax = vertices[0].x;
  *ymin = vertices[0].y;
  *ymax = vertices[0].y;
  for (size_t i = 1; i < vertex_count; i++) {
    *xmin = fmin(vertices[i].x, *xmin);
    *xmax = fmax(vertices[i].x, *xmax);
    *ymin = fmin(vertices[i].y, *ymin);
    *ymax = fmax(vertices[i].y, *ymax);
  }
}

Circle circumcenter(Vertex2 a, Vertex2 b, Vertex2 c) {
  // (x-xc)^2 + (y-yc)^2 = r^2
  // x^2 + y^2 - 2*x*xc - 2*y*yc +xc^2 +yc^2 - r^2 = 0
  // x^2 + y^2 + Dx + Ey + F = 0
  //
  // If we have 3 points, it suffices to solve for D, E, F:
  //
  // x1^2 + y1^2 + Dx1 + Ey1 + F = 0
  // x2^2 + y2^2 + Dx2 + Ey2 + F = 0
  // x3^2 + y3^2 + Dx3 + Ey3 + F = 0

  double delta = det3( //
      a.x, a.y, 1.0,   //
      b.x, b.y, 1.0,   //
      c.x, c.y, 1.0    //
  );
  double delta_D = det3(                  //
      -(a.x * a.x + a.y * a.y), a.y, 1.0, //
      -(b.x * b.x + b.y * b.y), b.y, 1.0, //
      -(c.x * c.x + c.y * c.y), c.y, 1.0  //
  );
  double delta_E = det3(                  //
      a.x, -(a.x * a.x + a.y * a.y), 1.0, //
      b.x, -(b.x * b.x + b.y * b.y), 1.0, //
      c.x, -(c.x * c.x + c.y * c.y), 1.0  //
  );
  double delta_F = det3(                  //
      a.x, a.y, -(a.x * a.x + a.y * a.y), //
      b.x, b.y, -(b.x * b.x + b.y * b.y), //
      c.x, c.y, -(c.x * c.x + c.y * c.y)  //
  );

  if (fabs(delta) < 1e-10f) {
    // degenerate triangle, return huge radius so it's always "bad"
    return (Circle){0, 0, 1e18f};
  }

  double D = delta_D / delta;
  double E = delta_E / delta;
  double F = delta_F / delta;

  double xc = -D / 2.0;
  double yc = -E / 2.0;
  double r = sqrtf(xc * xc + yc * yc - F);

  return (Circle){xc, yc, r};
}

void bowyer_watson_insert_point(Arena *a, TriangularMesh *mesh, int point_idx) {
  Vertex2 p = mesh->points[point_idx];

  size_t scratch = arena_save(a);

  size_t max_triangles = 2 * mesh->num_tris + 5;
  Edge *hole_edges = arena_alloc(a, max_triangles * sizeof(Edge));
  size_t num_hole_edges = 0;

  // Look for bad triangles
  int *bad = arena_alloc(a, max_triangles * sizeof(int));
  for (size_t i = 0; i < mesh->num_tris; i++) {
    Triangle *t = &mesh->tris[i];
    Circle cc = circumcenter(mesh->points[t->v[0]], mesh->points[t->v[1]],
                             mesh->points[t->v[2]]);
    // Is point inside the circumcircle?
    float dx = p.x - cc.x;
    float dy = p.y - cc.y;
    bad[i] = (dx * dx + dy * dy < cc.r * cc.r);
  }

  // Find the boundaries of the hole created by adding the new point.
  // The hole boundaries are the edges that aren't shared by two bad
  // triangles
  for (size_t i = 0; i < mesh->num_tris; i++) {
    if (!bad[i])
      continue;
    Triangle *t = &mesh->tris[i];

    // Current triangle edges:
    for (size_t e = 0; e < 3; e++) {
      Edge edge = {t->v[e], t->v[(e + 1) % 3]};
      int shared = 0;

      // Look for other triangles...
      for (size_t j = 0; j < mesh->num_tris; j++) {
        if (i == j || !bad[j])
          continue;
        Triangle *t2 = &mesh->tris[j];

        // Other triangle edges:
        for (size_t e2 = 0; e2 < 3; e2++) {
          Edge edge2 = {t2->v[e2], t2->v[(e2 + 1) % 3]};
          if (edge2.a == edge.a && edge2.b == edge.b ||
              edge2.a == edge.b && edge2.b == edge.a) {
            shared = 1;
            break;
          }
        }
        if (shared)
          break;
      }
      if (!shared) {
        if (num_hole_edges >= max_triangles) {
          fprintf(stderr, "hole_edges overflow\n");
          exit(1);
        }

        hole_edges[num_hole_edges++] = edge;
      }
    }
  }

  // Remove bad triangles
  size_t write = 0;
  for (size_t i = 0; i < mesh->num_tris; i++) {
    if (!bad[i]) {
      mesh->tris[write++] = mesh->tris[i];
    }
  }
  mesh->num_tris = write;

  // Triangles from new point to hole edges
  for (size_t i = 0; i < num_hole_edges; i++) {
    if (mesh->num_tris >= max_triangles) {
      fprintf(stderr, "tris overflow\n");
      exit(1);
    }
    int a = hole_edges[i].a;
    int b = hole_edges[i].b;
    int c = point_idx;
    mesh->tris[mesh->num_tris++] = (Triangle){a, b, c};
  }

  arena_restore(a, scratch);
}

void bowyer_watson_triangulation(Arena *a, TriangularMesh *mesh) {
  // Create a super triangle
  size_t num_points = mesh->num_points;
  int s0 = num_points;
  int s1 = num_points + 1;
  int s2 = num_points + 2;

  float xmin, xmax, ymin, ymax;
  get_vertices_extrema(mesh->points, mesh->num_points, &xmin, &xmax, &ymin,
                       &ymax);
  float w = xmax - xmin;
  float h = ymax - ymin;
  float margin = 10.0f * fmaxf(w, h);
  mesh->points[s0] = (Vertex2){xmin - margin, ymin - margin};
  mesh->points[s1] = (Vertex2){xmax + margin, ymin - margin};
  mesh->points[s2] = (Vertex2){(xmin + xmax) / 2, ymin + margin};

  mesh->tris[0] = (Triangle){s0, s1, s2};
  mesh->num_tris = 1;

  // Insert the actual points

  // Randomize order
  for (size_t i = mesh->num_points - 1; i > 0; i--) {
    size_t j = rand() % (i + 1);
    Vertex2 temp = mesh->points[i];
    mesh->points[i] = mesh->points[j];
    mesh->points[j] = temp;
  }
  for (size_t i = 0; i < mesh->num_points; i++) {
    bowyer_watson_insert_point(a, mesh, i);
  }

  // Remove triangles connected to the super triangle
  size_t write = 0; // <- we're rewriting the array!
  for (size_t i = 0; i < mesh->num_tris; i++) {
    Triangle *t = &mesh->tris[i];
    if (t->v[0] >= (int)num_points || //
        t->v[1] >= (int)num_points || //
        t->v[2] >= (int)num_points) {
      continue;
    }
    mesh->tris[write++] = *t;
  }
  mesh->num_tris = write;
}

void build_half_edge_mesh(TriangularMesh *mesh, HalfEdgeMesh *he) {

  for (size_t i = 0; i < mesh->num_tris; i++) {
    Triangle *t = &mesh->tris[i];
    size_t base = he->num_half_edges;

    for (size_t e = 0; e < 3; e++) {
      HalfEdge *h = &he->half_edges[base + e];
      h->face = i;
      h->origin = t->v[e];
      h->twin = -1;
      h->next = base + (e + 1) % 3;

      he->vert_edge[t->v[e]] = base + e;
    }

    he->face_edges[i] = base;
    he->num_half_edges += 3;
  }

  // Find twin half-edges

  for (size_t i = 0; i < he->num_half_edges; i++) {
    if (he->half_edges[i].twin != -1)
      continue;

    int a = he->half_edges[i].origin;
    int b = he->half_edges[he->half_edges[i].next]
                .origin; // Origin of the next half-edge
    for (size_t j = i + 1; j < he->num_half_edges; j++) {
      if (he->half_edges[j].twin != -1)
        continue;
      int c = he->half_edges[j].origin;
      int d = he->half_edges[he->half_edges[j].next]
                  .origin; // Origin of the next half-edge

      // a -> b
      // d <- c
      if (a == d && b == c) {
        he->half_edges[j].twin = i;
        he->half_edges[i].twin = j;
        break;
      }
    }
  }
}

// =====================================
//
// Boundary conditions
//
// =====================================

typedef enum { BC_WALL, BC_INLET, BC_OUTLET, BC_SYMMETRY } boundary_t;

typedef struct {
  boundary_t type;
  int half_edge;
  union {
    struct {
      float p_total;
      float T_total;
      float direction; // Flow angle in radians
    } inlet_bc;
    struct {
      float p_back;
    } outlet_bc;
  } as;
} BoundaryCondition;

typedef struct {
  BoundaryCondition *bcs;
  size_t count;
} MeshBoundaryConditions;

MeshBoundaryConditions create_mesh_boundary_conditions(Arena *a,
                                                       size_t num_edges) {
  MeshBoundaryConditions bcs = {0};
  bcs.bcs = arena_alloc(a, num_edges * sizeof(BoundaryCondition));
  memset(bcs.bcs, 0, num_edges * sizeof(BoundaryCondition));
  return bcs;
}

// =====================================
//
// Finite volumes
//
// =====================================

typedef struct {
  float rho;
  float u; // x velocity component
  float v; // y velocity component
  float p;
} FlowState;

typedef struct {
  float rho;
  float rho_u;
  float rho_v;
  float E;
} ConservativeState;

typedef struct {
  size_t num_cells;
  FlowState *states;
  float *areas;
} FVSolverData;

typedef struct {
  // Normals
  float nx;
  float ny;

  float L;
} EdgeInfo;

typedef struct {
  float mass;
  float momentum_x;
  float momentum_y;
  float energy;
} Flux;

ConservativeState prim_to_conservative(FlowState *s, float gamma) {
  ConservativeState cs;
  cs.rho = s->rho;
  cs.rho_u = s->rho * s->u;
  cs.rho_v = s->rho * s->v;
  cs.E = s->p / (gamma - 1.0) + s->rho * (s->u * s->u + s->v * s->v) / 2;
  return cs;
}

FlowState conservative_to_prim(ConservativeState *s, float gamma) {
  FlowState fs;
  fs.rho = s->rho;
  fs.u = s->rho_u / s->rho;
  fs.v = s->rho_v / s->rho;
  float E = s->E;

  // For an ideal gas: E = p/(gamma-1) + 1/2*rho(u^2 + v^2)
  //                   E(gamma-1) = p + (gamma-1)/2*rho(u^2 + v^2)
  //                   E(gamma-1) - (gamma-1)/2*rho(u^2 + v^2) = p

  fs.p = (gamma - 1.0) * (E - s->rho * (fs.u * fs.u + fs.v * fs.v) / 2);
  return fs;
}

Flux calculate_flux_euler(FlowState *s, float nx, float ny, float gamma) {
  // Warning: the normal components nx, ny need to be normalized
  // Euler flux.
  // Let v be some vector (vx, vy), and n be the normal vector (nx, ny)
  // Then, the projection of v onto n is simply v.n = vx * nx + vy * ny

  float un = s->u * nx + s->v * ny;
  float E = s->p / (gamma - 1.0) + s->rho * (s->u * s->u + s->v * s->v) / 2;

  Flux f = {.mass = s->rho * un,
            .momentum_x = s->rho * s->u * un + s->p * nx,
            .momentum_y = s->rho * s->v * un + s->p * ny,
            .energy = (E + s->p) * un};

  return f;
}

// Defaulting to Rusanov for now. Maybe add other later?
Flux rusanov_flux(FlowState *left, FlowState *right, float nx, float ny,
                  float gamma) {
  Flux flux_left = calculate_flux_euler(left, nx, ny, gamma);
  Flux flux_right = calculate_flux_euler(right, nx, ny, gamma);

  ConservativeState cs_left = prim_to_conservative(left, gamma);
  ConservativeState cs_right = prim_to_conservative(right, gamma);

  // Norms of normal velocities
  float un_left = fabsf(left->u * nx + left->v * ny);
  float un_right = fabsf(right->u * nx + right->v * ny);

  float sound_speed_left = sound_speed(left->rho, gamma, left->p);
  float sound_speed_right = sound_speed(right->rho, gamma, right->p);

  float s_max = fmax(un_left + sound_speed_left, un_right + sound_speed_right);

  Flux flux;
  flux.mass = (flux_left.mass + flux_right.mass) / 2 -
              s_max * (cs_right.rho - cs_left.rho) / 2;
  flux.momentum_x = (flux_left.momentum_x + flux_right.momentum_x) / 2 -
                    s_max * (cs_right.rho_u - cs_left.rho_u) / 2;
  flux.momentum_y = (flux_left.momentum_y + flux_right.momentum_y) / 2 -
                    s_max * (cs_right.rho_v - cs_left.rho_v) / 2;
  flux.energy = (flux_left.energy + flux_right.energy) / 2 -
                s_max * (cs_right.E - cs_left.E) / 2;
  return flux;
}

FlowState stagnation_state(BoundaryCondition *inlet, float poisson) {
  float R_air = R / M_air;
  float rho = gas_density(R_air, inlet->as.inlet_bc.p_total,
                          inlet->as.inlet_bc.T_total);
  return (FlowState){
      .v = 0.0f, .u = 0.0f, .p = inlet->as.inlet_bc.p_total, .rho = rho};
}

void init_solver(FVSolverData *solver, TriangularMesh *mesh) {
  solver->num_cells = mesh->num_tris;

  // Initialize the domain to ambient room conditions
  float R_air = R / M_air;
  float p_ambient = 100000.0f; // 1 atm
  float T_ambient = 300.0f;    // Room temp
  float rho_ambient = gas_density(R_air, p_ambient, T_ambient);

  FlowState s0 = {.rho = rho_ambient, .u = 0.0f, .v = 0.0f, .p = p_ambient};

  for (size_t i = 0; i < mesh->num_tris; i++) {
    Triangle *t = &mesh->tris[i];
    Vertex2 a = mesh->points[t->v[0]];
    Vertex2 b = mesh->points[t->v[1]];
    Vertex2 c = mesh->points[t->v[2]];

    solver->areas[i] = triangle_area(a, b, c);
    solver->states[i] = s0;
  }
}

EdgeInfo get_edge_info(Vertex2 *a, Vertex2 *b) {
  Vertex2 v = {.x = b->x - a->x, .y = b->y - a->y};
  float dx = v.x;
  float dy = v.y;
  float L = sqrtf(dx * dx + dy * dy);

  float nx = v.y / L; // Normalization
  float ny = -v.x / L;

  EdgeInfo info = {nx, ny, L};
  return info;
}

void calculate_edge_fluxes(FVSolverData *solver, TriangularMesh *mesh,
                           HalfEdgeMesh *he, BoundaryCondition *inlet,
                           float gamma, ConservativeState *residuals) {
  memset(residuals, 0, solver->num_cells * sizeof(ConservativeState));

  // Interior edges
  for (size_t i = 0; i < he->num_half_edges; i++) {
    HalfEdge *h = &he->half_edges[i];
    if (h->twin == -1)
      continue;
    if (h->twin < i)
      continue; // Already processed this pair

    int cell_left = h->face;
    int cell_right = he->half_edges[h->twin].face;

    Vertex2 a = mesh->points[h->origin];
    Vertex2 b = mesh->points[he->half_edges[h->next].origin];
    float L = segment_length(a, b);

    EdgeInfo info = get_edge_info(&a, &b);
    Flux flux =
        rusanov_flux(&solver->states[cell_left], &solver->states[cell_right],
                     info.nx, info.ny, gamma);

    float fA;

    fA = flux.mass * L;
    residuals[cell_left].rho += fA;
    residuals[cell_right].rho -= fA;

    fA = flux.momentum_x * L;
    residuals[cell_left].rho_u += fA;
    residuals[cell_right].rho_u -= fA;

    fA = flux.momentum_y * L;
    residuals[cell_left].rho_v += fA;
    residuals[cell_right].rho_v -= fA;

    fA = flux.energy * L;
    residuals[cell_left].E += fA;
    residuals[cell_right].E -= fA;
  }
}

void calculate_boundary_fluxes(FVSolverData *solver, TriangularMesh *mesh,
                               HalfEdgeMesh *he, MeshBoundaryConditions *bcs,
                               float gamma, ConservativeState *residuals) {
  for (size_t i = 0; i < bcs->count; i++) {
    BoundaryCondition *bc = &bcs->bcs[i];
    HalfEdge *h = &he->half_edges[bc->half_edge];

    int cell_left = h->face; // The interior cell
    FlowState state_left = solver->states[cell_left];

    Vertex2 a = mesh->points[h->origin];
    Vertex2 b = mesh->points[he->half_edges[h->next].origin];
    float L = segment_length(a, b);
    EdgeInfo info = get_edge_info(&a, &b);

    Flux flux = {0};

    if (bc->type == BC_WALL) {
      FlowState ghost = state_left;

      // Calculate the normal velocity (V . n)
      float vn = state_left.u * info.nx + state_left.v * info.ny;

      // Reflect the velocity vector across the normal: V_ghost = V - 2*(V.n)*n
      // This creates an equal and opposite collision at the wall!
      ghost.u = state_left.u - 2.0f * vn * info.nx;
      ghost.v = state_left.v - 2.0f * vn * info.ny;

      // Run it through Rusanov to get the stabilized flux!
      flux = rusanov_flux(&state_left, &ghost, info.nx, info.ny, gamma);
    } else if (bc->type == BC_INLET) {
      // Ghost cell
      FlowState ghost = stagnation_state(bc, gamma);
      flux = rusanov_flux(&state_left, &ghost, info.nx, info.ny, gamma);
    } else if (bc->type == BC_OUTLET) {
      // Ghost cell
      FlowState ghost = state_left;
      ghost.p = bc->as.outlet_bc.p_back;
      flux = rusanov_flux(&state_left, &ghost, info.nx, info.ny, gamma);
    }

    residuals[cell_left].rho += flux.mass * L;
    residuals[cell_left].rho_u += flux.momentum_x * L;
    residuals[cell_left].rho_v += flux.momentum_y * L;
    residuals[cell_left].E += flux.energy * L;
  }
}

void euler_step(Arena *a, FVSolverData *solver, TriangularMesh *mesh,
                HalfEdgeMesh *he, MeshBoundaryConditions *bcs, float gamma,
                float dt) {

  size_t scratch = arena_save(a);
  ConservativeState *residuals =
      arena_alloc(a, solver->num_cells * sizeof(ConservativeState));

  calculate_edge_fluxes(solver, mesh, he, NULL, gamma, residuals);

  calculate_boundary_fluxes(solver, mesh, he, bcs, gamma, residuals);

  for (size_t i = 0; i < solver->num_cells; i++) {

    FlowState old_prim = solver->states[i];
    ConservativeState U = prim_to_conservative(&old_prim, gamma);

    float factor = dt / solver->areas[i];

    U.rho -= factor * residuals[i].rho;
    U.rho_u -= factor * residuals[i].rho_u;
    U.rho_v -= factor * residuals[i].rho_v;
    U.E -= factor * residuals[i].E;

    solver->states[i] = conservative_to_prim(&U, gamma);
  }

  arena_restore(a, scratch);
}

// =====================================
//
// Nozzle-specific
//
// =====================================

float nozzle_radius(float L, float x) {
  float throat_x = 1.5f;
  float r_inlet = 1.0f;
  float r_throat = 0.4f;
  float r_exit = 1.2f;

  if (x <= throat_x) {
    float t = x / throat_x;
    // cosine interpolation: smooth at both ends
    float s = 0.5f * (1.0f - cosf(t * M_PI));
    return r_inlet + s * (r_throat - r_inlet);
  } else {
    float t = (x - throat_x) / (L - throat_x);
    float s = 0.5f * (1.0f - cosf(t * M_PI));
    return r_throat + s * (r_exit - r_throat);
  }
}

PointCloud sample_points_in_nozzle(Arena *a, float L, size_t nx, size_t ny) {

  PointCloud cloud = create_point_cloud(a, 2 * nx * ny);

  float dx = L / (1.0f * (nx - 1));
  float dy = L / (1.0f * (ny - 1));

  float throat_x = 1.5f; // throat location
  float r_inlet = 1.0f;
  float r_throat = 0.4f;
  float r_exit = 1.2f;

  for (size_t i = 0; i < nx; i++) {
    float x = L * (i + 0.5f) / nx;
    float r = nozzle_radius(L, x);
    for (size_t j = 0; j < nx; j++) {
      float y = -r + (2.0f * r) * (j + 0.5f) / ny;

      // Margin
      if (fabsf(y) < r * 0.95f) {
        float jitter = 0.01f * (2.0f * r / ny); // tiny bit of spacing
        float random_number = ((float)rand() / (float)RAND_MAX - 0.5f);
        cloud.points[cloud.point_count++] =
            (Vertex2){x + jitter * random_number, y + jitter * random_number};
      }
    }
  }

  // Add more points to the inlet and outlet

  for (size_t j = 0; j < ny; j++) {
    float r = nozzle_radius(L, 0.0f);
    float y = -r + (2.0f * r) * (j + 0.5f) / ny;

    // Margin
    if (fabsf(y) < r * 0.95f) {
      cloud.points[cloud.point_count++] = (Vertex2){0, y};
    }
  }

  for (size_t j = 0; j < ny; j++) {
    float r = nozzle_radius(L, L);
    float y = -r + (2.0f * r) * (j + 0.5f) / ny;

    // Margin
    if (fabsf(y) < r * 0.95f) {
      cloud.points[cloud.point_count++] = (Vertex2){L, y};
    }
  }

  return cloud;
}

void apply_boundary_conditions_to_nozzle(float L, HalfEdgeMesh *he,
                                         Vertex2 *points,
                                         MeshBoundaryConditions *bcs) {
  bcs->count = 0;

  for (size_t i = 0; i < he->num_half_edges; i++) {
    HalfEdge *h = &he->half_edges[i];
    if (h->twin != -1) {
      continue;
    }

    // It's a wall by default. Could also be an inlet or outlet
    BoundaryCondition bc;
    bc.type = BC_WALL;
    bc.half_edge = i;

    Vertex2 origin = points[h->origin];
    Vertex2 target = points[he->half_edges[h->next].origin];
    float mx = (origin.x + target.x) / 2.0;

    float tol = 0.001;
    if (mx < tol) {
      bc.type = BC_INLET;
      bc.as.inlet_bc.direction = 0;
      bc.as.inlet_bc.p_total = 3e5;  // Pa
      bc.as.inlet_bc.T_total = 3000; // K
    }
    if (mx > L - tol) {
      bc.type = BC_OUTLET;
      bc.as.outlet_bc.p_back = 1e5; // Pa
    }

    // printf("Pushing %zu / %d...\n", i, MAX_HALF_EDGES);
    bcs->bcs[bcs->count++] = bc;
  }
}

float calculate_dt(FVSolverData *solver, float gamma) {
  float min_dt = 1e30f;
  float cfl = 0.4f; // Safe Courant number

  for (size_t i = 0; i < solver->num_cells; i++) {
    FlowState s = solver->states[i];
    float c = sound_speed(s.rho, gamma, s.p);
    float v_mag = sqrtf(s.u * s.u + s.v * s.v);
    float max_wave_speed = v_mag + c;

    // Approximate characteristic length of the triangle (sqrt of area)
    float dx = sqrtf(solver->areas[i]);

    float cell_dt = cfl * dx / max_wave_speed;
    if (cell_dt < min_dt) {
      min_dt = cell_dt;
    }
  }
  return min_dt;
}

// =====================================
//
// Visualization
//
// =====================================

void compute_viewport(Vertex2 *vertices, size_t vertex_count, int screen_w,
                      int screen_h, int padding_x, int padding_y, float *scale,
                      float *offset_x, float *offset_y) {
  float xmin = 0;
  float xmax = 0;
  float ymin = 0;
  float ymax = 0;
  get_vertices_extrema(vertices, vertex_count, &xmin, &xmax, &ymin, &ymax);

  float world_w = xmax - xmin;
  float world_h = ymax - ymin;

  float avail_w = screen_w - 2 * padding_x;
  float avail_h = screen_h - 2 * padding_y;

  *scale = fmin(avail_w / world_w, avail_h / world_h);

  float cx = (xmin + xmax) / 2;
  float cy = (ymin + ymax) / 2;

  // We need an offset s.t. cx * scale + offset = screen_w/2 (center of the
  // screen) Same for height! Thus: offset = screen_w/2 - cx * scale
  *offset_x = screen_w / 2.0f - cx * (*scale);
  *offset_y = screen_h / 2.0f - cy * (*scale);
}

Color mach_to_color(float mach) {
  // Simple colormap from blue (Mach 0) to red (Mach 2+)
  float t = fminf(fmaxf(mach / 2.0f, 0.0f), 1.0f);
  unsigned char r = (unsigned char)(255 * t);
  unsigned char b = (unsigned char)(255 * (1.0f - t));
  return (Color){r, 0, b, 255};
}

// =====================================
//
// WebAssembly API
//
// =====================================

static Arena a;
static TriangularMesh mesh;
static HalfEdgeMesh he;
static MeshBoundaryConditions bcs;
static FVSolverData solver;

static float *mach_numbers;
static float current_gamma = 1.4f;

EMSCRIPTEN_KEEPALIVE
void init_wasm_simulation() {
  a = arena_create(10 * 1024 * 1024); // 10 MB

  Polygon poly = create_polygon(&a, 1024);

  float L = 4.0;
  for (float x = 0; x <= L; x += 0.1) {
    float r = nozzle_radius(L, x);
    Vertex2 v_up;
    v_up.x = x;
    v_up.y = r;
    poly.vertices[poly.vertex_count++] = v_up;
  }
  for (float x = L; x >= 0; x -= 0.1) {
    float r = nozzle_radius(L, x);
    Vertex2 v_down;
    v_down.x = x;
    v_down.y = -r;
    poly.vertices[poly.vertex_count++] = v_down;
  }

  PointCloud cloud = sample_points_in_nozzle(&a, L, 30, 30);
  for (size_t i = 0; i < poly.vertex_count; i++) {
    cloud.points[cloud.point_count++] = poly.vertices[i];
  }

  mesh = create_triangular_mesh(&a, 4 * cloud.point_count);
  mesh.points = cloud.points;
  mesh.num_points = cloud.point_count;
  bowyer_watson_triangulation(&a, &mesh);

  // Centroid culling for now
  size_t write = 0;
  for (size_t i = 0; i < mesh.num_tris; i++) {
    Triangle *t = &mesh.tris[i];
    float cx = (mesh.points[t->v[0]].x + mesh.points[t->v[1]].x +
                mesh.points[t->v[2]].x) /
               3.0f;
    float cy = (mesh.points[t->v[0]].y + mesh.points[t->v[1]].y +
                mesh.points[t->v[2]].y) /
               3.0f;

    if (fabsf(cy) < nozzle_radius(L, cx) + 1e-4) {
      mesh.tris[write++] = mesh.tris[i];
    }
  }
  mesh.num_tris = write;

  he = create_half_edge_mesh(&a, mesh.num_points, mesh.num_tris);
  build_half_edge_mesh(&mesh, &he);

  bcs = create_mesh_boundary_conditions(&a, he.num_half_edges / 2);
  apply_boundary_conditions_to_nozzle(L, &he, mesh.points, &bcs);

  solver.states = arena_alloc(&a, mesh.num_tris * sizeof(FlowState));
  solver.areas = arena_alloc(&a, mesh.num_tris * sizeof(float));
  init_solver(&solver, &mesh);

  mach_numbers = arena_alloc(&a, mesh.num_tris * sizeof(float));

  EM_ASM({ window.cfdBuffer = HEAP8.buffer; });
}

EMSCRIPTEN_KEEPALIVE
void step_wasm_simulation(int steps) {
  for (int i = 0; i < steps; i++) {
    float dt = calculate_dt(&solver, current_gamma);
    euler_step(&a, &solver, &mesh, &he, &bcs, current_gamma, dt);
  }

  for (size_t i = 0; i < mesh.num_tris; i++) {
    FlowState s = solver.states[i];
    float v_mag = sqrtf(s.u * s.u + s.v * s.v);
    float c = sound_speed(s.rho, current_gamma, s.p);
    mach_numbers[i] = v_mag / c;
  }
}

EMSCRIPTEN_KEEPALIVE
void set_inlet_pressure(float pressure_pa) {
  for (size_t i = 0; i < bcs.count; i++) {
    if (bcs.bcs[i].type == BC_INLET) {
      bcs.bcs[i].as.inlet_bc.p_total = pressure_pa;
    }
  }
}

EMSCRIPTEN_KEEPALIVE
void set_inlet_temperature(float temp_k) {
  for (size_t i = 0; i < bcs.count; i++) {
    if (bcs.bcs[i].type == BC_INLET) {
      bcs.bcs[i].as.inlet_bc.T_total = temp_k;
    }
  }
}

EMSCRIPTEN_KEEPALIVE
void reset_wasm_simulation() {
  float R_air = 8.314f / 0.029f;
  float p_ambient = 100000.0f; // 1 atm
  float T_ambient = 300.0f;    // Room temp
  float rho_ambient = p_ambient / (R_air * T_ambient);

  FlowState s0 = {.rho = rho_ambient, .u = 0.0f, .v = 0.0f, .p = p_ambient};

  for (size_t i = 0; i < solver.num_cells; i++) {
    solver.states[i] = s0;
  }
}

EMSCRIPTEN_KEEPALIVE int get_num_tris() { return mesh.num_tris; }
EMSCRIPTEN_KEEPALIVE int get_num_points() { return mesh.num_points; }
EMSCRIPTEN_KEEPALIVE Vertex2 *get_points_ptr() { return mesh.points; }
EMSCRIPTEN_KEEPALIVE Triangle *get_tris_ptr() { return mesh.tris; }
EMSCRIPTEN_KEEPALIVE float *get_mach_ptr() { return mach_numbers; }
