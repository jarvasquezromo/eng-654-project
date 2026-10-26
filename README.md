# ENG-654 Project 1A: GoFa kinematics lab

Interactive pages for **Project 1A: Branch-Resolved Singularity Analysis and Path Placement on ABB GoFa** (EPFL ENG-654, Kinematics-Grounded Motion Planning for Robots). The assignment is in [`docs/project_1A.pdf`](docs/project_1A.pdf).

Each part of the assignment gets its own page: a 3D view (or map) plus one side panel with tabs, so only one topic is on screen at a time.

| Step | Page | Status |
|---|---|---|
| 1 | Kinematic model: URDF → D–H table and screw axes | ✅ |
| 2 | FK validation: URDF vs D–H vs PoE, plus a check against the supplied IK solver | ✅ |
| 3 | Screw Jacobian, ³J₅, conditioning, finite-difference verification | ✅ |
| 4 | (q₂, q₃) conditioning slice with classified singular curves | ✅ |
| 5 | IK validation and persistent branches | next |
| 6 | Closed contour and placement family | next |
| 7 | Branch-resolved feasibility and failure maps | next |
| 8 | Nearest vs branch-aware continuation | next |

## Run locally

The pages use ES modules, so serve the folder rather than opening the files directly:

```sh
python3 -m http.server 8000
# open http://localhost:8000
```

Numerical checks (Node 18+):

```sh
node tests/check.mjs
```

## Model summary

Standard D–H, `Rz(θ) Tz(d) Tx(a) Rx(α)`, with `θi = qi + θi,0`. Frame 6 is `tool0`.

| i | θi,0 | d (m) | a (m) | α |
|---|---|---|---|---|
| 1 | 0 | 0.265 | 0 | −π/2 |
| 2 | −π/2 | 0 | 0.444 | 0 |
| 3 | 0 | 0 | 0.110 | −π/2 |
| 4 | 0 | 0.470 | 0 | π/2 |
| 5 | 0 | 0 | 0.080 | −π/2 |
| 6 | π | 0.101 | 0 | 0 |

URDF, D–H and PoE agree to about 1e-15 on random configurations. The screw Jacobian matches central differences to about 1e-10, and all 8 configurations in the solver's example reproduce the target pose to about 1e-15.

## Layout

```
index.html            home page with the step cards
pages/                one HTML page per step
js/math.js            linear algebra, SE(3), Jacobi eigen-solver
js/gofa.js            URDF parser, D–H, PoE, Jacobians, conditioning
js/viewer.js          three.js viewer (STL meshes on the URDF chain)
js/ui.js              header, steps, tabs, sliders
js/pages/             page scripts
assets/abb_gofa/      URDF and STL meshes from the course assets
data/                 supplied solver example output
tests/check.mjs       headless numerical cross-checks
vendor/three/         three.js r180 (MIT)
```

Robot assets come from the [ENG-654 course materials](https://epfl-lasa.github.io/eng-654/). three.js is © three.js authors, MIT licence.
