# ENG-654 Project 1A: GoFa kinematics lab

**Live site: https://jarvasquezromo.github.io/eng-654-project/** (no install, works on phones).

Interactive pages for **Project 1A: Branch-Resolved Singularity Analysis and Path Placement on ABB GoFa** (EPFL ENG-654, Kinematics-Grounded Motion Planning for Robots). The assignment is in [`docs/project_1A.pdf`](docs/project_1A.pdf).

Each part of the assignment gets its own page: a 3D view (or map) plus one side panel with tabs, so only one topic is on screen at a time.

| Step | Page | Status |
|---|---|---|
| 1 | Kinematic model: URDF → D–H table and screw axes | ✅ |
| 2 | FK validation: URDF vs D–H vs PoE, plus a check against the supplied IK solver | ✅ |
| 3 | Screw Jacobian, ³J₅, conditioning, finite-difference verification | ✅ |
| 4 | Conditioning slices: (q₂, q₃) with classified singular curves, and task-space xy / xz / yz planes at any offset, tool orientation and tool length | ✅ |
| 5 | IK validation (our FK, periodic angles, limits) and persistent branches along the contour | ✅ |
| 6 | Closed full-pose contour and its 2-D placement family, two tool orientations | ✅ |
| 7 | Branch-resolved feasibility and failure maps (no IK / limit / discontinuity / margin), nearest vs branch-aware | ✅ |
| 8 | Nearest vs branch-aware continuation | next |
| + | 3R atlas from Lecture 06: det Jₚ, critical curves, IK counts, cusps (course presets and GoFa's arm) | ✅ |

## Open it

The site is served by GitHub Pages straight from `main` (root folder), so every push to `main` updates it within a minute or two. Nothing to build.

To run it locally instead, serve the folder (the pages use ES modules, so opening the files directly does not work):

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

### Steps 5–7 in short

- **Contour**: ellipse (semi-axes a, b) in the plane perpendicular to the global tool direction R_g; the tool tilts ±θ and spins ±ψ periodically, so it is a closed 6-D pose path, sampled at N points.
- **Placements**: the contour centre moves over an n × n grid in x, y at fixed height z; repeated for orientation A and B (defaults: tool down, tool forward).
- **Feasibility**: each start configuration at s = 0 is followed around the loop. Nearest continuation fails on the first of: no IK, joint step > Δq_max, joint limit (after unwrapping), 1/κ < margin. Branch-aware search uses a layered graph of all in-limit, in-margin configurations and succeeds if the start pose is reached again.
- Settings are shared between Steps 5–7 in the browser (localStorage). Step 7 runs in web workers; a 19 × 19 grid takes about 15 s on a laptop.

The browser IK (`js/ik.js`) fixes q₆, solves the arm (q₁, q₂, q₃) in closed form for the frame-4 origin, and finds the q₆ roots of z₃ · z₄ = 0 per arm branch. On 400 random poses it matched the supplied `crb15000-ik` solver on 2947 configurations, missed 3 (all at the elbow reach boundary) and added none.

URDF, D–H and PoE agree to about 1e-15 on random configurations. The screw Jacobian matches central differences to about 1e-10, and all 8 configurations in the solver's example reproduce the target pose to about 1e-15.

## Layout

```
index.html            home page with the step cards
pages/                one HTML page per step
js/math.js            linear algebra, SE(3), Jacobi eigen-solver
js/gofa.js            URDF parser, D–H, PoE, Jacobians, conditioning
js/ik.js              all-solutions IK in the browser (q6 sweep + closed-form arm)
js/atlas.js           3R atlas maths: det Jp grid, zero set, critical curves, cusps, meridian IK
js/tasks.js           contour, placement grid, candidates, branch tracking, both continuation strategies
js/feas-worker.js     web worker that evaluates placements for the Step 7 maps (js/pool.js runs several)
js/taskui.js, plot.js drawing helpers for Steps 5–7
js/viewer.js          three.js viewer (STL meshes on the URDF chain)
js/ui.js              header, steps, tabs, sliders
js/pages/             page scripts
assets/abb_gofa/      URDF and STL meshes from the course assets
data/                 supplied solver example output
tests/check.mjs       headless numerical cross-checks
vendor/three/         three.js r180 (MIT)
```

Robot assets come from the [ENG-654 course materials](https://epfl-lasa.github.io/eng-654/). three.js is © three.js authors, MIT licence.
