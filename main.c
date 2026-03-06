#include <stddef.h>
#define TINY_LA_IMPLEMENTATION
#include "raylib.h"
#include "tinyla.h"
#include <stdio.h>

typedef struct {
  float x;
  float y;
} Vertex2;

#define MAX_VERTICES_PER_POLYGON 1024
typedef struct {
  Vertex2 vertices[MAX_VERTICES_PER_POLYGON];
  size_t vertex_count;
} Polygon;

void displayPoly(Polygon *poly) {
  InitWindow(800, 450, "Gaph viz");

  while (!WindowShouldClose()) {
    BeginDrawing();
    ClearBackground(DARKGRAY);

    for (size_t i = 1; i < poly->vertex_count + 1; i++) {
      size_t prev = i - 1;
      size_t cur = i % poly->vertex_count;
      float scale = 100.0f;
      float offsetX = 100.0f;
      float offsetY = 100.0f;

      DrawLine(poly->vertices[prev].x * scale + offsetX,
               poly->vertices[prev].y * scale + offsetY,
               poly->vertices[cur].x * scale + offsetX,
               poly->vertices[cur].y * scale + offsetY, WHITE);
    }

    EndDrawing();
  }

  CloseWindow();
}

int main() {

  Polygon poly = {0};

  Vertex2 vs[4] = {{.x = 0.0, .y = 0.0},
                   {.x = 0.0, .y = 1.0},
                   {.x = 1.0, .y = 1.0},
                   {.x = 1.0, .y = 0.0}};

  for (size_t i = 0; i < 4; i++) {
    poly.vertices[poly.vertex_count++] = vs[i];
  }

  displayPoly(&poly);

  return 0;
}
