<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# Scene-graph approaches for object/room relationships — research

**Issue #138.** The maintainer's reframe (2026-09-22): after semantic segmentation
we have class labels → instances and rooms clustered separately, but no way to ask
"which objects are in which room" or "which objects belong together" (chairs paired
with a desk; chair clusters forming distinct seating arrangements distinguished by
proximity). The goal is abstraction from points → objects → relations, and a
recommendation on how to get there.

Research date: 2026-09-22. SceneGraph stub as of commit fa6be94;
ProjectDB schema v14 (Morton metadata, no schema bump); Hydra v0.5.1;
3DSSG dataset v1; IFC 4.3 (ISO 16739:2024).

> **Scope.** This document is research only — no schema changes, no
> implementation. It maps the existing codebase state, surveys the academic and
> industry landscape, argues a pragmatic v1 design, and proposes a staged plan.

---

## 1. What exists in-repo today

### 1.1 The SceneGraph stub

`libs/reusex/include/reconstruction/SceneGraph.hpp` declares a class but
**has no corresponding `.cpp`** — every public method (`extract`, `get_labels`,
`save`, `export`, `load`, `patch_segmentation`, `planar_region_growing`,
`project_labels_from_database`, `segment_rooms`) is unimplemented. Zero files
in `libs/`, `apps/`, or `tests/` include it; the class has no consumers and no
tests.

The internal data model declared in the header:

| Type | Fields | Notes |
|---|---|---|
| `SceneNodeType` | `point_cluster`, `plane`, `object` | enum class |
| `PointCluster` | `vector<int> point_indices` | raw indices only |
| `ScenePlane` | `Vector4d coefficients`, `Vector3d origin` | |
| `SceneObject` | `int label` | no centroid, no bbox |
| `SceneVertexData` | `Vector3d centroid`, `SceneNodeType type`, `variant<PointCluster,ScenePlane>` | |
| `SceneEdgeData` | *(empty struct)* | no relation attributes |

The graph is a `boost::adjacency_list<vecS,vecS,undirectedS,SceneVertexData,SceneEdgeData>`.
The empty `SceneEdgeData` is the critical gap: the header sketches nodes but has
no vocabulary for edges (relation types, weights, directionality, provenance).

`Registry.hpp` is a fully implemented template property-map store (header-only)
that `SceneGraph` inherits from, but it likewise has zero consumers.

**Conclusion:** the stub encodes the right intuition (nodes + edges on a Boost
graph) but is too incomplete to extend; it should be redesigned based on
the model recommended in §4.

### 1.2 Instances

`segmentation/segment_instances.hpp` → `SegmentInstancesResult`:
- `CloudLPtr instance_labels` — per-point instance ID (uint32_t, 0 = unlabeled)
- `map<uint32_t, uint32_t> instance_to_semantic` — instance → class
- `map<uint32_t, size_t> instance_sizes` — instance → point count

`core/ProjectDB.hpp` → `InstanceRecord`:
- `uint32_t instance_id`, `std::string guid`, `int semantic_class`, `int point_count`

```sql
-- instances table (ProjectDB.cpp ~line 821)
CREATE TABLE IF NOT EXISTS instances (
  cloud_id       INTEGER NOT NULL REFERENCES point_clouds(id) ON DELETE CASCADE,
  instance_id    INTEGER NOT NULL,
  guid           TEXT NOT NULL UNIQUE,
  semantic_class INTEGER NOT NULL DEFAULT -1,
  point_count    INTEGER NOT NULL DEFAULT 0,
  PRIMARY KEY (cloud_id, instance_id)
);
```

**What is absent:** no centroid, no bounding box, no room assignment, no
relations to other instances. Point membership is stored only as per-point labels
in the `"instances"` `CloudL` blob — not as an explicit index set.

### 1.3 Rooms

`segmentation/segment_rooms.hpp` → `segment_rooms()` returns a `CloudLPtr`:
a per-point room label cloud produced by Leiden community clustering over the
plane adjacency graph (via igraph). A room is identified solely by its label
integer. **There is no room metadata struct**, no area, centroid, polygon,
floor-plan footprint, or room table in ProjectDB. The only persistent artifact
is the `"rooms"` label cloud stored in `point_clouds` / `point_cloud_data`.

### 1.4 ComponentRecord and the existing hierarchy

`core/ComponentRecord` (stored in `building_components`):

```cpp
struct ComponentRecord {
  std::string name, guid, type;    // type: "window" | "door" | "wall"
  std::vector<uint8_t> vertex_data; // CoplanarPolygon packed float64 xyz triples
  std::array<double, 4> plane;     // Hessian normal form
  int parent_id = -1;              // FK to building_components.id (no enforced FK)
  double confidence = -1.0;
  std::string metadata;            // JSON; includes source_instance_guid
  std::string notes;
};
```

`parent_id` supports one level of hierarchy (e.g. window inside wall). The
`source_instance_guid` in `metadata` gives provenance back to the originating
instance. However, there is no mechanism to query "all components inside room X",
and `ComponentRecord` is limited to detected architectural features — chairs,
desks, and furniture groups are not represented.

### 1.5 Pose-graph edges

`pose_graph_edges` (schema v15) stores sensor-frame relations:

```sql
CREATE TABLE IF NOT EXISTS pose_graph_edges (
  id           INTEGER PRIMARY KEY,
  from_node_id INTEGER NOT NULL,   -- sensor_frames.node_id
  to_node_id   INTEGER NOT NULL,
  edge_type    TEXT    NOT NULL,   -- "odometry" | "loop_closure" | "panorama"
  residual     REAL    NOT NULL,
  weight       REAL
);
```

This pattern — a typed edge table with two node references and optional weight —
is directly reusable for semantic object relations (§4.1).

### 1.6 Label class names

There is no hardcoded label→class mapping in the library. The mapping is
model-driven and stored per-project in `pipeline_log` as `"annotate_class_map"`
(JSON `{int: string}`), then written to `label_definitions` for the `"labels"`
cloud. The ONNX default prompts include `"chair"` (class 7) and `"desk"` (class
12 as "desk lamp" — note: `"table"` is class 6, `"desk"` in the TensorRT prompts
does not appear directly, but a project can use any class map via `rux create
annotate`). The class-name→class-id lookup requires a `label_definitions` join,
which the existing C++ API supports.

### 1.7 What relations are already derivable

The table below maps queries from the maintainer's examples against what the
current codebase supports without schema changes:

| Query | Status | How |
|---|---|---|
| All instances with semantic_class = N | ✅ | SQL on `instances`; `db.instances("instances")` |
| Instance by class name ("chair") | ✅ | Join `instances` + `label_definitions` on `"labels"` cloud |
| Material linked to instance X | ✅ | `db.instance_material_guid("instances", X)` |
| All windows / doors / walls | ✅ | `db.list_building_components(type)` |
| Which room does instance X belong to? | ❌ | No stored relation; must load full `"rooms"` + `"instances"` CloudL and iterate |
| Centroid of instance X | ❌ | Not stored; compute from `"cloud"` + `"instances"` CloudL |
| Bounding box of instance X | ❌ | Same — not stored |
| Room polygon / footprint | ❌ | Does not exist anywhere in the codebase |
| Chair instances near desk Y | ❌ | Requires centroids + distance query |
| Group: seating arrangement | ❌ | Requires centroid + proximity clustering |

**Summary:** everything about *points and labels* is queryable; everything about
*objects and their spatial relationships* is absent.

---

## 2. The 3D scene-graph literature

### 2.1 Academic systems

#### Armeni et al. — 3D Scene Graph (ICCV 2019)

Proposes a hierarchical attributed graph for building interiors with four layers:
building → room → object → camera. Nodes carry typed attributes (room:
floor_area, scene_category, volume; object: class, action_affordance, material,
location, size; camera: FOV, pose). Spatial containment is encoded as
structural data attributes (`object.parent_room`, `room.parent_building`) —
rule-derived from 3D bounding-box containment, not learned. However, the node
*content* (object detection + attribute annotation) uses Mask R-CNN and other
learned models before assembling the graph.

Code: `StanfordVL/3DSceneGraph` (MIT). **No C++ runtime:** the graph is produced
offline from Gibson/Matterport meshes via a Python pipeline. Cannot be used as
a drop-in C++ component. Dataset scope is similar to ReUseX (interior scans of
real buildings), making it the closest academic precedent for our domain.

**Relevance:** the four-layer hierarchy and the typed per-node attribute schema
are the right conceptual target. The graph schema (not the code) is the
reference.

#### Wald et al. — 3DSSG (CVPR 2020)

3D Semantic Scene Graphs on ScanNet: object-instance nodes with WordNet-
hierarchical class labels; edges in three categories — support ("standing on",
"lying in", "hanging from"), proximity ("close by", "same part", "same as"),
and comparative ("bigger than", "same material"). All relations are learned via
a GCN with per-class binary cross-entropy loss, using PointNet node features
and relative-position/size edge features. 41 predicates total, 534 object
classes.

A verified key constraint: proximity edges are only computed between nodes that
share a support parent — a bottle on a table has no proximity relation to a
chair, but the supporting table does. This hierarchically filters proximity
edges, which is directly relevant for chair↔desk pairing (both must share a
floor/room support parent before a `close_by` edge is generated).

Code: `ShunChengWu/3DSSG` (MIT for training code). **Caveat on data:** the
3DSSG *dataset* annotations may carry a non-commercial Creative Commons license
(ScanNet data license is CC BY-NC-SA 4.0); only the C++ training code was
confirmed MIT. GNN weights trained on this data would carry the data license
restriction. Do not use pre-trained 3DSSG weights in ReUseX without verifying
the data license chain.

**Relevance:** the 22-relation vocabulary and the support-filtered proximity
model are the right end-state for a richer relation graph. For a v1, only three
types are needed: *contains* (room → object), *close_by* (proximity), *same
part* (group membership) — all achievable by geometric rules (§3).

#### Hughes et al. — Hydra (Science Robotics 2022)

Real-time incremental 3D scene graph from a live SLAM system. Five confirmed
layers: metric-semantic mesh → objects/agents → places (GVD-derived free-space
nodes) → rooms → buildings. Structural containment (object-in-room, room-in-
building) is rule-derived via geometric query; room segmentation uses
`clusterGraphByModularity()` on the places graph — no neural network in the
room-segmentation path. Optional learned room *classification* (semantic label
for the room) is a separate external step in `Hydra-GNN`, not part of the core
Hydra graph construction.

Code: `MIT-SPARK/Hydra` (BSD-2-Clause ✅, GPL-3.0-compatible) and the
underlying graph data structure `MIT-SPARK/Spark-DSG` (BSD-2-Clause ✅, also
available as PyPI package `spark-dsg`, actively maintained through 2025). Hydra
requires ROS 2 and the full Kimera semantic SLAM stack; **not a drop-in
library.** Spark-DSG's C++ data structure alone is a reusable directed scene
graph (node layers, typed edges, attributes) — the closest existing GPL-
compatible C++ library to what SceneGraph.hpp aims to be.

**Relevance:** confirms that a five-layer hierarchy is the practical target for
indoor environments, and that object-in-room containment is a geometric rule
universally. Spark-DSG's typed-layer graph data structure is the most directly
reusable prior art: it exposes `LayerId`, `SceneGraphNode`, `SceneGraphEdge`
concepts that could inform the SceneGraph redesign (§4.3).

#### Wu et al. — SceneGraphFusion (CVPR 2021)

Online scene graph prediction from RGB-D frames at 35 Hz, incremental. Nodes
are 3D object instances; edges are 8 learned support-type predicates (standing
on, attached to, hanging on, connected to, part of, built in, same part,
supported by) trained on 20 NYUv2 object classes from ScanNet/3RScan via a GNN
with PointNet features. Not applicable to ReUseX's offline pipeline model.

Code: `ShunChengWu/SceneGraphFusion` — confirmed **BSD-2-Clause** ✅ (a
CC BY-NC-SA claim appearing in some search summaries was identified as a
hallucination; the LICENSE file in the repository is BSD-2-Clause). GPL-
compatible for reference.

**Relevance:** demonstrates that learned GNN inference degrades when training
data is limited. For a building-reuse corpus of few dozen scans, geometric
rules outperform learned models for v1.

#### Werby et al. — HOV-SG (RSS 2024)

Hierarchical Open-Vocabulary 3D Scene Graph with a four-level containment-only
hierarchy: building root → floor → room → object. All inter-node relations are
rule-derived from geometric containment (no learned edge predicates). Open-
vocabulary object class labels come from CLIP/DINO image embeddings, not a
fixed class set. Floor and room segmentation is geometric (height slicing +
horizontal connectivity).

Code: `hovsg/HOV-SG` (MIT ✅). Research-grade Python; no C++ library. Directly
relevant as the clearest recent evidence that geometric containment rules are
sufficient for building-scale scene graphs, and that a fixed class vocabulary is
unnecessary.

**Relevance:** the strongest evidence that a v1 should use rules, not learned
models, for containment. The floor/room segmentation approach (height slicing
for floors, horizontal connectivity for rooms) is an alternative to Leiden
clustering worth noting for `segment_rooms.hpp`.

### 2.2 Industry/standard formats

#### IFC — Industry Foundation Classes (ISO 16739:2024, IFC 4.3)

The domain-exact reference model for a building-reuse project. IFC's spatial
hierarchy uses **two distinct relation types that must not be mixed**:

```
IfcProject
  └── IfcSite
        └── IfcBuilding
              └── IfcBuildingStorey          (floor)
                    ├── IfcRelAggregates     ← decomposes the spatial structure itself
                    └── IfcSpace             (room)
                          └── IfcRelContainedInSpatialStructure
                                └── IfcFurniture / IfcFlowTerminal / ...  (objects)
```

`IfcRelAggregates` composes the spatial structure itself (Site → Building →
Storey → Space); `IfcRelContainedInSpatialStructure` places physical elements
*inside* a spatial structure element (element → Space). The critical constraint
confirmed by the IFC 4.3 spec: **`IfcRelContainedInSpatialStructure` is
one-to-one** — an element can be contained in exactly one spatial structure
element at a time. This means "chair is in room A" is an exclusive assignment,
not a many-to-many relation.

Key relation types for ReUseX:

| IFC relation | Meaning | Analogue in ReUseX |
|---|---|---|
| `IfcRelContainedInSpatialStructure` | element contained in exactly one space | instance in room (one-to-one) |
| `IfcRelAggregates` | spatial decomposition (storey → spaces, building → storeys) | room→floor→building hierarchy |
| `IfcRelAssociatesMaterial` | object has a material | instance→material (already exists in ProjectDB) |
| `IfcRelAssignsToGroup` | objects belong to a functional group | seating arrangement / furniture layout |
| `IfcRelConnectsElements` | elements are physically connected | window in wall (`ComponentRecord.parent_id`) |

The `IfcRelAssignsToGroup` / `IfcGroup` pattern for seating arrangements is
exactly what the maintainer describes: a group with a semantic type ("seating
arrangement") and a set of member objects (chairs). This is IFC's standard
mechanism for furniture layout and directly maps to the `group_member` relation
type in §4.1.

**One-to-one containment** aligns with the majority-vote room assignment in §3.1:
a chair is assigned to the room where the plurality of its points fall, not
distributed across multiple rooms. For objects straddling room boundaries (long
corridor benches), the IFC model requires a tie-breaking rule — majority vote
satisfies this.

**Library:** `ifcOpenShell` (LGPL-3.0-or-later ✅ compatible with GPL-3.0-or-later)
provides a C++ and Python IFC API. Adding it as a dependency for internal
representation would be heavy — but the IFC *data model* is the right conceptual
template even if the implementation uses plain SQLite tables. The existing
`export_scene.hpp` / Speckle export could later emit IFC-conformant hierarchy
without a deep coupling.

#### USD — Universal Scene Description (Pixar / ASWF)

A hierarchical scene description with prim/property model and typed schemas
(`UsdGeom`, `UsdShade`, `UsdSkel`). USD's strength is runtime composition
(layers, variants, references) for VFX/game production. It has no semantic
spatial-relation layer — there is no equivalent of `IfcRelContainedInSpatialStructure`
or `IfcRelAssignsToGroup`. Object-in-room would require a custom schema.

License: Apache-2.0 ✅. The `OpenUSD` C++ library is large (~200 MB), adds
significant build complexity, and provides nothing for the spatial-relation
problem that SQLite tables cannot provide more simply. Not recommended as a
dependency.

#### glTF (Khronos)

A JSON scene graph (nodes, meshes, materials, cameras, skins) designed as a
rendering exchange format. Nodes form a transform hierarchy; there is no semantic
grouping or spatial-containment layer. Extensible via `EXT_` schemas, but no
standard extension covers object-in-room relations.

glTF is already the *output* target for the GUI frontend (three.js scenes). It
is not a candidate for the internal relation representation.

### 2.3 Comparison summary

| System | Layers | Edge vocabulary | Relation source | C++ library | License | Use in ReUseX |
|---|---|---|---|---|---|---|
| Armeni 3D Scene Graph | 4 (building/room/object/camera) | containment, co-occurrence | structure: rule; node content: partly learned (Mask R-CNN) | None (Python dataset) | MIT (code) | Schema only |
| 3DSSG | 1 (objects) | 41 predicates in 3 categories | learned GNN (PointNet++) | None (Python dataset + training) | MIT (code); data: CC BY-NC-SA | Relation vocab as target |
| SceneGraphFusion | 1 (objects) | 8 support predicates | learned GNN | Yes (C++, 35 Hz) | BSD-2-Clause ✅ | Schema ref; online-only |
| Hydra / Spark-DSG | 5 (mesh/objects/places/rooms/buildings) | containment, traversability | structure: rule; room class: optional learned | Yes (BSD-2-Clause ✅) | BSD-2-Clause ✅ | Best C++ data-structure reference |
| HOV-SG | 4 (root/floor/room/object) | containment only | all rule (geometry) | None (Python) | MIT ✅ | Design pattern reference |
| IFC 4.3 | 5 (project/site/building/storey/space + elements) | containment, aggregation, assignment, grouping | rule (authorship) | ifcOpenShell (LGPL ✅) | ISO/buildingSMART | Data model template; IFC export target |
| USD | 1 (prim hierarchy) | composition arcs | n/a (rendering) | OpenUSD (Apache-2.0 ✅) | Apache-2.0 | Not applicable |
| glTF | 1 (node/mesh) | transform hierarchy | n/a (rendering) | None needed | royalty-free | Output target only |

---

## 3. Rule-based vs learned relation inference

For the maintainer's three concrete examples, the argument for rule-based
inference is strong:

### 3.1 "Which objects are in which room"

**Geometric rule:** for each instance, iterate the (already loaded) `"rooms"` and
`"instances"` CloudL blobs — both are index-aligned to `"cloud"`. For each point
where `instances[i].label == X`, record `rooms[i].label`. Take the majority vote
(or require >50% consensus) → that is the room for instance X.

This is O(N_cloud) per scan, done once at `rux create relations` time and cached
in a new `instance_spatial` table. No ML, no polygon geometry, no room footprint
needed. The approach matches Hydra's object-in-room rule (centroid falls inside
room volume) but uses the existing per-point label clouds instead of a volumetric
room model, which ReUseX does not yet have.

**Limitation:** a large object straddling two rooms (e.g. a long corridor bench)
may be ambiguously assigned. Accept the majority-vote result for v1; add a
`room_confidence` field for future disambiguation.

### 3.2 "Chairs paired with their desk"

**Geometric rule:** compute centroid of each chair instance and each desk/table
instance from the `"cloud"` + `"instances"` CloudL. Build a k-d tree over desk
centroids. For each chair centroid, find the nearest desk centroid within a
configurable radius (default: 1.5 m). If found, emit a `paired_with` relation
(chair_guid, desk_guid).

No ML needed. The chair↔desk relation degenerates gracefully: if no desk is
within radius, the chair is part of a seating arrangement (§3.3). Orientation
(chair faces the desk) adds precision but is not required for a v1 — proximity
alone is sufficient to cluster most office configurations.

**Learned alternative:** GNN-based relation prediction (3DSSG / SceneGraphFusion)
can infer "sitting on" / "in front of" from point features. However, it requires
training data labelled with IFC-style furniture relations in interior scans —
which does not exist for building-reuse datasets. For a corpus of a few dozen
scans, training a GNN is not viable; geometric rules will generalize better.

### 3.3 "Seating arrangement" grouping

**Geometric rule:** DBSCAN or single-linkage clustering over chair instance
centroids (within the same room), with `eps` = 2–3 m and `min_samples` = 2.
Each cluster is a seating-arrangement group. Emit `group_member` relations (each
chair_guid → group_guid) plus a `group_type = "seating_arrangement"` label on
the group node.

Distinguishing "chairs around a conference table" from "chairs in a corridor"
requires checking whether a desk/table is inside the cluster's convex hull — a
straightforward addition to the DBSCAN pass.

### 3.4 Verdict

All three examples in the maintainer's description are achievable with geometric
rules over existing instance + room CloudL data, with no ML inference, no new
model weights, and no additional dependencies beyond what is already in the build.
Learned relation inference (3DSSG-style GNN) is the correct long-term target for
richer relation vocabularies (22 types), but requires labelled training data that
does not exist for this domain today. Start with rules; add a learned backend
when a training corpus is available.

---

## 4. Data-model recommendation

### 4.1 Two new tables in ProjectDB (additive, no existing table changed)

**`instance_spatial`** — caches per-instance geometry derived from the CloudL:

```sql
CREATE TABLE IF NOT EXISTS instance_spatial (
  cloud_id    INTEGER NOT NULL REFERENCES point_clouds(id) ON DELETE CASCADE,
  instance_id INTEGER NOT NULL,
  cx REAL, cy REAL, cz REAL,          -- centroid
  bmin_x REAL, bmin_y REAL, bmin_z REAL,  -- axis-aligned bbox min
  bmax_x REAL, bmax_y REAL, bmax_z REAL,  -- axis-aligned bbox max
  room_label  INTEGER NOT NULL DEFAULT -1, -- from majority-vote containment
  PRIMARY KEY (cloud_id, instance_id)
);
```

Populated by a new `rux create relations --spatial` pass. Invalidated and
regenerated whenever `create instances` or `create rooms` reruns (foreign key
`ON DELETE CASCADE` handles the cloud side; a pipeline stage check handles the
dependency ordering).

**`scene_relations`** — typed, directed relations between scene entities:

```sql
CREATE TABLE IF NOT EXISTS scene_relations (
  id           INTEGER PRIMARY KEY,
  from_guid    TEXT NOT NULL,  -- guid of source entity (instance, component, group)
  to_guid      TEXT NOT NULL,  -- guid of target entity
  relation_type TEXT NOT NULL, -- 'contains', 'paired_with', 'group_member', 'adjacent_to'
  group_id     INTEGER,        -- for multi-member groups (seating arrangements)
  weight       REAL,           -- optional: proximity distance, confidence, etc.
  metadata     TEXT            -- opaque JSON for relation-type-specific attributes
);
CREATE INDEX IF NOT EXISTS idx_sr_from ON scene_relations(from_guid);
CREATE INDEX IF NOT EXISTS idx_sr_to   ON scene_relations(to_guid);
CREATE INDEX IF NOT EXISTS idx_sr_type ON scene_relations(relation_type);
```

This mirrors the `pose_graph_edges` pattern (typed edge table, two node
references, optional weight) but operates over GUIDs (stable across schema
migration) rather than integer node IDs. The GUID convention means room labels —
which currently have no GUID — will need one; room GUIDs can be minted at
`create relations` time and stored as a JSON mapping in `pipeline_log`.

### 4.2 Where this lives in the module graph

`STANDARDS.md §1` layering: `core` ↔ `segmentation` ↔ `reconstruction` ↔
`io`. The new tables belong in `core/ProjectDB` (persistence layer — no new
module). The computation (centroid, containment, proximity clustering) belongs
in a new `create relations` pipeline stage, implemented in `apps/rux/src/create/relations.cpp`
calling library functions in `libs/reusex/src/reconstruction/` or a new
`libs/reusex/src/scene/` sub-module under `reconstruction`.

### 4.3 The SceneGraph class — recommended disposition

**Do not extend the current stub; redesign it as a thin in-memory query facade
over ProjectDB.**

Proposed revised interface (replaces the current Boost.Graph + Registry design):

```cpp
namespace reusex::reconstruction {

class SceneGraph {
public:
  explicit SceneGraph(ProjectDB& db);

  // Object-in-room containment
  std::vector<InstanceRecord> instances_in_room(int room_label) const;
  int room_of(std::string_view instance_guid) const;

  // Relations
  std::vector<SceneRelation> relations_of(std::string_view entity_guid,
                                          std::string_view relation_type = "") const;
  std::vector<SceneRelation> group(int group_id) const;

  // Spatial
  std::vector<InstanceRecord> instances_near(
      Eigen::Vector3d const& point, double radius_m,
      int semantic_class = -1) const;
};

} // namespace reusex::reconstruction
```

This class does not persist anything — it wraps SQL queries over
`instance_spatial` and `scene_relations` with a convenient C++ API. No
Boost.Graph, no Registry inheritance, no HDF5. The `build_scene_graph(db)`
free function in `stages.hpp` triggers the computation stage; `SceneGraph(db)`
just opens the read view.

The internal Boost adjacency_list can be kept as a private member if traversal
algorithms (shortest path, BFS over object groups) are needed later, but it is
not required for the v1 use cases.

### 4.4 Consistency with the 2026-09-21 audit

The audit (comment on #138) found that "persistence belongs in ProjectDB, not in
a separate graph file." This recommendation is consistent: the two new SQLite
tables ARE ProjectDB. The `SceneGraph` class is an in-memory query facade, not a
separate persistence format. HDF5, Boost serialization, and custom graph files
are not introduced.

---

## 5. Staged plan

### Stage 1 — Instance spatial index and room containment (~1 PR, ~1 week)

**Goal:** answer "which objects are in which room" end-to-end.

- Add `instance_spatial` table to ProjectDB (new schema version).
- Add `rux create relations` subcommand with `--spatial` mode:
  loads `"cloud"`, `"instances"`, `"rooms"` CloudL; computes centroids, bboxes,
  majority-vote room assignment; writes `instance_spatial`.
- Add `db.instance_spatial(cloud_name, instance_id)` and
  `db.instances_in_room(cloud_name, room_label)` to ProjectDB API.
- Expose via CLI: `rux get instances --json` adds `centroid` and `room_label`
  fields; `rux get instances --room <label>` filters by room.
- Python bindings: extend `InstanceRecord` Python type with `centroid` and `room_label`.

**Acceptance criteria:**
- Unit test: synthetic `ProjectDB` with 3 rooms and 10 instances; assert
  `instances_in_room(1)` returns the correct subset.
- Integration test: run on a real scan; verify every instance has a non-(-1)
  `room_label` after `rux create relations`.
- Performance: `rux create relations` completes under 60 s for a 10 M-point scan
  (sequential CloudL blob scan; no GPU needed).

**Exposure:** CLI (`rux get instances --room N`), JSON API endpoint in `ruxd`
(`GET /api/v1/instances?room=N`), Python bindings.

### Stage 2 — Proximity relations and grouping (~1 PR, ~1 week)

**Goal:** chair↔desk pairing and seating-arrangement grouping.

- Add `scene_relations` table to ProjectDB.
- Add `rux create relations --proximity` mode:
  builds k-d tree over instance centroids (from `instance_spatial`);
  emits `paired_with` relations for chair↔table pairs within configurable
  `--pair-radius` (default 1.5 m);
  runs DBSCAN over same-class centroids within each room to emit
  `group_member` relations with `group_id` for seating arrangements.
- Add `db.scene_relations(from_guid)` and `db.scene_group(group_id)` to
  ProjectDB API.
- Expose via CLI: `rux get relations --instance <guid>` lists all relations;
  `rux get relations --type paired_with` lists all pairings.

**Acceptance criteria:**
- Unit test: synthetic instances with known positions; assert DBSCAN produces
  the expected groups.
- Integration test: run on a scan with chairs and tables; verify chair→desk
  pairings exist and are spatially plausible (visual check via `rux render`).
- `rux create relations --spatial --proximity` runs sequentially after
  `create instances` and `create rooms`; `rux validate --stage relations` checks
  prerequisites.

### Stage 3 — GUI scene-tree panel (~1 PR, ~2 weeks)

**Goal:** render the object/room hierarchy in the React frontend.

- Add `GET /api/v1/scene/rooms` endpoint to `ruxd`: returns array of rooms with
  `room_label`, `instance_count`, `instance_guids[]`.
- Add `GET /api/v1/scene/relations?from_guid=X` for relation graph traversal.
- Implement a collapsible scene-tree panel in the GUI (React, three.js):
  rooms as top-level nodes → instances as children → relations as in-panel links.
- Scene tree selection → highlight the corresponding points in the 3D viewport
  (existing frustum-culled tile fetch, filter by `instance_guid` mask).

**Acceptance criteria:**
- GUI shows all rooms with their contained instances.
- Selecting a seating-arrangement group highlights all member chairs in the 3D view.
- Pausing the scene-tree panel does not cause a regression in the point-stream
  tile fetch (vitest + end-to-end test).

### Stage 4 — IFC export of spatial relations (~1 PR, ~1 week)

**Goal:** emit `IfcRelContainedInSpatialStructure` and `IfcRelAssignsToGroup`
in the Rhino/IFC export path.

- The existing Speckle and Rhino exports (`io/speckle.hpp`, `io/rhino.hpp`)
  currently flatten all building components; extend them to include
  room→instance containment and furniture groups from `scene_relations`.
- No new dependency required for Speckle export (JSON). Rhino export via
  OpenNURBS does not natively support IFC; document as a known limitation.
- A future `rux export ifc` subcommand could use `ifcOpenShell` (LGPL-3.0,
  GPL-compatible) to emit a valid IFC 4.3 file with spatial containment — out of
  scope for this stage but the data model (§4.1) is designed to support it.

### Stage 5 — Relation quality metrics and `rux analyze relations` (~1 PR, ~1 week)

**Goal:** validate that computed relations are plausible and provide feedback
to the user.

- `rux analyze relations`: reports per-room instance counts, pairing coverage
  (fraction of chairs with a paired desk), ungrouped-instance fraction, and
  average centroid distance for `paired_with` relations.
- `rux validate --stage relations` checks that `instance_spatial` and
  `scene_relations` are populated and not stale relative to `instances` and
  `rooms`.

---

## 6. Open questions for the maintainer

1. **Room GUIDs.** Rooms currently have no stable GUID — they are identified
   solely by integer label. A GUID scheme (e.g. minted at `create relations`
   time and stored in `pipeline_log`) is needed for `scene_relations.from_guid`
   to reference rooms. Should room GUIDs be added to a new `rooms` table, or
   stored as a JSON map in `pipeline_log` (lighter, consistent with how
   `annotate_class_map` works)?

2. **Relation vocabulary.** The initial types proposed are `contains`,
   `paired_with`, `group_member`, `adjacent_to`. Are there other building-reuse
   domain relations that should be in scope for v1? (e.g. `overlooks` for windows
   → rooms, `abuts` for walls → rooms.)

3. **Delete or retain the SceneGraph stub?** Given that the redesigned class
   (§4.3) has a very different interface, the safest path is to delete
   `SceneGraph.hpp` (and `Registry.hpp` since it has no other consumers) and
   start fresh in Stage 1. The alternative — keeping the stub and refactoring
   in-place — risks confusion from the empty methods. The namespace mismatch
   (`reusex::geometry` declared but file lives in `reconstruction/`) also needs
   correcting regardless.

4. **Proximity thresholds.** The `--pair-radius 1.5 m` default for chair↔desk
   pairing and the `eps=2–3 m` for DBSCAN seating arrangements are estimates
   based on typical office furniture dimensions. Are there dataset-measured values
   from the building scans already available, or should these be tunable
   parameters with sensible defaults?

5. **Stage ordering vis-à-vis mesh generation.** Room polygons (useful for more
   accurate containment than the majority-vote approach) can be derived from the
   `CellComplex` mesh output. Should Stage 1 require `create mesh` to have run
   first, giving access to room volumes? Or should it run earlier (just after
   `create rooms`) using only the CloudL majority-vote approach? The latter is
   recommended for v1 to reduce the dependency chain.

---

## 7. Sources

### Academic

- Armeni et al., "3D Scene Graph: A Structure for Unified Semantics, 3D Space, and Camera", ICCV 2019: <https://www.semanticscholar.org/paper/3D-Scene-Graph:-A-Structure-for-Unified-Semantics,-Armeni-He/77b4f542ea56b02804672790b6482df400428c95>; code: <https://github.com/StanfordVL/3DSceneGraph> (MIT; Python pipeline, no C++ runtime)
- Wald et al., "Learning 3D Semantic Scene Graphs from 3D Indoor Reconstructions", CVPR 2020: <https://arxiv.org/abs/2004.03967>; dataset/project: <https://3dssg.github.io/> (training code MIT; dataset may be CC BY-NC-SA — verify before using weights)
- Hughes et al., "Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization", Science Robotics 2022: <https://arxiv.org/html/2201.13360v2>; code: <https://github.com/MIT-SPARK/Hydra> (BSD-2-Clause ✅); graph data structure: <https://github.com/MIT-SPARK/Spark-DSG> (BSD-2-Clause ✅; `pip install spark-dsg`)
- Wu et al., "SceneGraphFusion: Incremental 3D Scene Graph Prediction from RGB-D Sequences", CVPR 2021: <https://openaccess.thecvf.com/content/CVPR2021/papers/Wu_SceneGraphFusion_Incremental_3D_Scene_Graph_Prediction_From_RGB-D_Sequences_CVPR_2021_paper.pdf>; code: <https://github.com/ShunChengWu/SceneGraphFusion> (BSD-2-Clause ✅ — CC BY-NC-SA claim in some sources is a hallucination; verify LICENSE file before integrating)
- Werby et al., "Hierarchical Open-Vocabulary 3D Scene Graphs for Language-Grounded Robot Navigation", RSS 2024: <https://arxiv.org/html/2403.17846v2>; code: <https://github.com/hovsg/HOV-SG> (MIT ✅; Python only)

### Standards and formats

- IFC 4.3 (ISO 16739:2024): <https://standards.buildingsmart.org/IFC/RELEASE/IFC4_3/>
- `IfcRelContainedInSpatialStructure` (one-to-one containment): <https://ifc43-docs.standards.buildingsmart.org/IFC/RELEASE/IFC4x3/HTML/lexical/IfcRelContainedInSpatialStructure.htm>
- ifcOpenShell C++ / Python IFC library (LGPL-3.0-or-later ✅): <https://ifcopenshell.org/>, <https://github.com/IfcOpenShell/IfcOpenShell>
- OpenUSD (Apache-2.0 ✅): <https://openusd.org/release/index.html>
- glTF 2.0 spec (Khronos, royalty-free): <https://github.com/KhronosGroup/glTF>

### In-codebase references

- SceneGraph stub: `libs/reusex/include/reconstruction/SceneGraph.hpp`
- Registry: `libs/reusex/include/reconstruction/Registry.hpp`
- Instance segmentation: `libs/reusex/include/segmentation/segment_instances.hpp`, `reconcile_instances.hpp`
- Room segmentation: `libs/reusex/include/segmentation/segment_rooms.hpp`
- ComponentRecord: `libs/reusex/include/core/component_record.hpp`
- Label semantics: `libs/reusex/include/core/label_semantics.hpp`
- Pose graph edges: `libs/reusex/src/core/ProjectDB.cpp` (~line 1205)
- Pipeline stage template: `apps/rux/src/create/planes.cpp`
- Related: #138 (this issue), #265 (GUI plan), #267 (agent gateway / MCP)
