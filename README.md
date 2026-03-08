# Tiny CFD: Supersonic Nozzle Simulation in WebAssembly

**[Live Browser Demo](https://marciorvneto.github.io/tiny-cfd/)**

A 2D compressible Euler solver written entirely from scratch in C. This engine models transient supersonic fluid flow through a Converging-Diverging (CD) nozzle using the Finite Volume Method (FVM). It is designed to run natively on desktop or compile to WebAssembly for zero-friction browser execution.

## Engine Architecture

- **Mathematics:** Solves the compressible Euler equations using a Rusanov (Local Lax-Friedrichs) flux scheme. Uses ghost cells for reflective solid boundary conditions.
- **Meshing:** Features a custom Delaunay triangulator and a half-edge data structure for fast adjacency queries.
- **Memory:** Built on a custom arena allocator. In the browser build, this contiguous memory block is shared directly with JavaScript via `Float32Array` views for blazingly fast, zero-copy rendering at 60+ FPS.

## Build Instructions

The project has two distinct build targets: a native desktop application (using Raylib for visualization) and a headless WebAssembly engine (using Emscripten).

### 1. Native Desktop Build (`main.c`)

Compiles the engine and renders it using Raylib.

**Dependencies:** `gcc`, `make`, and Raylib (included in `vendor/`).

```bash
make
./out/main

```

### 2. WebAssembly Build (`browser.c`)

Compiles the headless physics engine to WASM and generates the JavaScript glue code. The output is routed to the `docs/` folder to be served statically by GitHub Pages.

**Dependencies:** Emscripten (`emcc`).

```bash
make browser

```

To run locally, spin up a web server in the root directory (e.g., `python3 -m http.server 8000`) and navigate to `/docs`.

## About

Built by Márcio, a Chemical Engineer with a PhD in process simulation and optimization. This project serves as a foundational proof-of-concept for the high-performance, engineering calculations powering the diagramming tools at [Voima](https://voimatoolbox.com/en-beta).
