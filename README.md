
# SurveillanceBot — 2‑D SLAM, Occupancy‑Grid Navigation and Goal Execution

**Level:** BSc (Hons) Robotics / CS  
**Environment:** ROS Kinetic + Gazebo Turtlebot world supplied in `robot_assignment_ws`

## 1  Problem Definition
The robot must (i) explore the highlighted area, (ii) build a consistent 2‑D occupancy map, and (iii) on demand accept an `(x,y)` target from **stdin** and drive to it autonomously. Master’s‑only visual‑object detection is **out of scope** for the Honours brief.

## 2  Top‑Level Solution Strategy
| Stage | Best‑practice method | Rationale |
|-------|---------------------|-----------|
| **Mapping (SLAM)** | *slam_gmapping* (particle‑filter grid‑based SLAM) | Mature, real‑time, single‑beam‑LIDAR SLAM supported on Turtlebot; parameters are well documented and easy to tune. |
| **Localisation (run‑time)** | Ground‑truth pose from `/gazebo/get_model_state` | Acceptable per brief; avoids the drift and extra tuning required by AMCL. |
| **Path Planning** | A* on the saved occupancy grid | Deterministic, optimal with an admissible heuristic, fast on static 2‑D grids. |
| **Motion Control** | Pure‑Pursuit steering with PID‑bounded velocities | Handles curvature smoothly, tolerates waypoint sparsity, requires only two gains to tune. |

This combination minimises algorithmic complexity while meeting every deliverable (map, navigation, video) and leaving enough marks for parameter insight rather than code cleverness.

## 3  Detailed Methodology & Parameter Guidance

### 3.1  Mapping with *slam_gmapping*
| Parameter | Role | Recommended starting value | How/why to tune |
|-----------|------|---------------------------|-----------------|
| `linearUpdate` | Distance travelled before processing a scan | **0.30 m** | Lower ⇒ denser updates (better local accuracy) but heavier CPU. Increase if scans overlap too much and cause lag. |
| `angularUpdate` | Yaw change before a scan update | **0.15 rad** | Same trade‑off; keep ≈ ½ ° lidar angular res. |
| `minimumScore` | Min match score for scan acceptance | **100 000** | Raise if wrong loop closures appear; lower if genuine matches are rejected. |
| `particles` | No. of RBPF particles | **20** | More particles resist kidnapped‑robot effects but scale O(n). In this rectangular, low‑symmetry map 20–30 is enough. |
| `maxUrange` | Laser max range considered | **5.5 m** | Set equal to simulated sensor limit to avoid phantom walls. |
| `delta` | Map cell size | **0.05 m** | Matches Turtlebot footprint resolution; finer adds little benefit here. |
| `laserpz` / `lasersigma` | Likelihood model noise | **0.035 / 0.03** | Start with default; lower sigma if map seems “fuzzy”. |

**Tuning workflow**

1. Tele‑operate small loops; visually check map growth in RViz.  
2. If walls look “melted”, lower `linearUpdate`.  
3. If scans sometimes vanish (big grey areas), `minimumScore` is too high.  
4. When satisfied, run `map_saver` once and disable Gmapping for all navigation runs.

### 3.2  Occupancy‑Grid Post‑processing
* Threshold: treat cells with `data ≥ 50` as obstacles.  
* Inflate obstacles by **1.5× robot radius** (≈0.25 m) to create a configuration‑space buffer; otherwise A* may graze corners.

### 3.3  A* Path Planner
* **Heuristic**: Euclidean distance `h = √(dx² + dy²)` (admissible).  
* **Cost of step**: 1 for cardinal moves, √2 for diagonals.  
* **Tie‑breaking**: favour lower `g` cost to encourage straight paths.  
* **Grid resolution sensitivity**: doubling `delta` halves node count and plan time but blurs narrow doorways—stick to 0.05 m.

### 3.4  Pure‑Pursuit Motion Controller
| Parameter | Meaning | Start with | Guidelines |
|-----------|---------|-----------|------------|
| Look‑ahead distance `L_d` | Arc target ahead of base | **0.30 m** | Increase in narrow corridors → smoother; decrease in clutter → precision. |
| Max linear vel `v_max` | Safety & stability | **0.30 m s⁻¹** | Keep < Turtlebot spec 0.5 m s⁻¹; raise only if schedule demands. |
| Max angular vel `ω_max` | Turn rate limiter | **0.6 rad s⁻¹** | Prevents lidar aliasing and wheel‑slip. |
| PID gains (`K_p`, `K_d`) | Heading error correction | `K_p = 4`, `K_d = 0.5` | Tune by Ziegler‑Nichols on heading step response; leave `K_i = 0` (no steady‑state heading bias). |

### 3.5  Startup & TF Ordering
1. `./startWorld` (Gazebo, base controllers).  
2. Wait **5 s** for `/tf` tree stabilisation.  
3. Launch `slam_gmapping`.  
4. Launch navigation node **after** map exists.  

Avoids “extrapolation into the past” TF warnings—most common rookie error.

## 4  Validation & Performance Metrics
| Metric | Acceptable target | Measurement tool |
|--------|------------------|------------------|
| **Map completeness** | ≥ 95 % of highlighted area discovered | Overlay saved PGM on floor‑plan in Gimp; count white + black pixels. |
| **Localization error** | ≤ 0.10 m | Compare `/gazebo/model_states` ground‑truth with own pose. |
| **Path success rate** | 100 % to two user‑chosen goals | Record console + video. |
| **Mean translational overshoot** | < 0.15 m past final waypoint | Post‑analyse `/cmd_vel`. |

## 5  Risk Register & Mitigations
| Risk | Impact | Mitigation |
|------|--------|-----------|
| Gmapping divergence in long corridors | Robot “teleports” in map → unusable grid | Increase `particles`; enlarge `linearUpdate` so fewer correlated scans. |
| Path planner queues too many nodes | CPU lag → late `cmd_vel` | Downsample grid to 0.10 m **only** for planning; follow on 0.05 m map. |
| Lidar reflections create ghost obstacles | A* declares no path | Raise occupancy threshold to ignore sparse hits; re‑inflate after planning. |

## 6  Project Deliverables
1. `gmapping.launch` with tuned parameters.  
2. `navigator.launch` (starts path node **only**).  
3. `mapping_params.yaml`.  
4. One‑page PDF algorithm summary (A*, pure‑pursuit, parameters).  
5. ≤ 3 min screen‑capture video: mapping, two goal runs, terminal echo.

## 7  Indicative Schedule (10 days)
| Day | Activity |
|-----|-----------|
| 1 | ROS tutorials, workspace compile, simulator smoke‑test |
| 2–3 | Gmapping parameter sweep, save final map |
| 4 | Offline A* on saved map image |
| 5 | Integrate A* and Gazebo pose into ROS node |
| 6 | Implement pure‑pursuit controller |
| 7 | System test, parameter fine‑tuning |
| 8 | Record demo footage |
| 9 | Draft one‑page report |
| 10 | Buffer, troubleshooting, submission upload |
