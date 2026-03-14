# Implementation Plan

## 1. RTAB-MAP Dockerised Service
- **Base Image:** `enord-reconstruction:latest` (contains RTAB-MAP, OpenCV, PCL, ZeroMQ).
- **Functionality:** 
  - Connect to `tcp://trajectory:14000` via ZeroMQ SUB to receive timestamp, RGB, Depth, and Pose matrix.
  - Implement an internal buffer to hold all received frames and process them sequentially (accurate reconstruction, not real-time).
  - Feed data (RGB, Depth, Pose) into `rtabmap::Rtabmap` instance for mapping.
  - Expose the optimized, live internal point cloud to other services via ZeroMQ REP socket on port `15000`. The service will serialize the point cloud (x, y, z, r, g, b) into a binary payload upon request.
- **Implementation Style:** Clean C++ program (`reconstruction.cpp`), minimalist, no print statements/comments unless necessary, `CMakeLists.txt` for building, and a minimal `Dockerfile`.
- **Integration:** Add the `reconstruction` service to `docker-compose.yml`.

## 2. Godot Client Integration
- **Functionality:** `World.cs` will periodically poll the RTAB-MAP service (via ZeroMQ REQ to `tcp://127.0.0.1:15000`) for the full point cloud.
- **Performance:** Use a Compute Shader (`points.glsl`) to efficiently update a RenderingServer `MultiMesh` or direct vertex buffer, ensuring high frame rates even with millions of points.
  - `MultiMeshInstance3D` is the standard way to render millions of identical instances (e.g., points or small quads) in Godot efficiently.
  - Compute shader will map the byte array directly into the multimesh buffer.
- **Style:** Clean C# code, seamlessly integrated with existing tracking.

## 3. Checklist
- [x] Create `reconstruction` folder.
- [x] Write `reconstruction/CMakeLists.txt` and `reconstruction/Dockerfile`.
- [x] Write `reconstruction/reconstruction.cpp`.
- [x] Update `docker-compose.yml` with `reconstruction` service on port 15000.
- [x] Create `client/scenes/points.glsl` (compute shader).
- [x] Update `client/scripts/World.cs` to fetch point cloud from 15000 and push to compute shader.
- [ ] Test everything to ensure accurate and performant point cloud rendering without freezing the trajectory tracking.

---

# Investigation: No Point Cloud Rendering

## Theories
1. **ZeroMQ Data Receipt:** The C# client isn't receiving the data from the `reconstruction` service, or the data size is 0 bytes. Is the RTAB-MAP service actually generating a point cloud?
2. **Compute Shader Execution Issue:** The data is received, but the compute shader logic writes all zeroes, or the buffer extraction `BufferGetData` is failing and returning an empty buffer.
3. **MultiMesh Buffer Format:** Godot MultiMesh 3D buffer format for `Transform3D` might expect a different memory layout than what we are providing. The shader produces 16 floats per instance: `[X.x, X.y, X.z, Y.x, Y.y, Y.z, Z.x, Z.y, Z.z, O.x, O.y, O.z, R, G, B, A]`. If this layout is wrong, the instances might have scale 0 or invalid positions (NaNs).
4. **Visibility / Scale / Position:** The points might actually be rendering, but they are either at `(0,0,0)` overlapping, too small (size `0.05`), or far outside the camera's view. We should add a temporary debug log to check the bounds of the point cloud or at least its point count.
5. **RTAB-MAP Pose Conversion:** We're passing the pose from `trajectory.cpp` (an `Eigen::Matrix4f`) straight into `rtabmap::Transform`. Is this conversion correct? If not, `rtabmap` could be dropping frames due to invalid odometry/poses, resulting in an empty global map.

## Investigation Plan
1. Add a small temporary debug log in `World.cs` to see if `rawBytes.Length` is greater than 0, and how many points are being pushed to the shader.
2. If `pointCount == 0`, investigate the `reconstruction.cpp` service to see if it's successfully receiving data and extracting points.
3. If `pointCount > 0`, debug the output buffer of the compute shader to see if the transformed floats are correct (no NaNs, sensible origin coordinates).
4. Verify the `MultiMesh` format expected by Godot by referencing standard C# examples or experimenting with standard `MeshInstance` transforms.

## Findings
- **Theory 3 Confirmed:** The `Transform3D` memory layout produced by the compute shader was incorrectly structured as a 4x4 matrix (with origin elements assigned to `Basis.Y.X`, `Basis.Z.Y`, etc.) rather than the contiguous Godot 3x4 `[Basis.X, Basis.Y, Basis.Z, Origin]` layout.
  - *Fix Applied:* Modified `points.glsl` to write the elements exactly to `out_offset + 0` through `out_offset + 8` for the identity Basis vectors, and `+9`, `+10`, `+11` for `px`, `py`, `pz`.
- **GLSL Macro Issue:** Godot's runtime SPIR-V compiler failed to parse the `#[compute]` macro which is only valid for Editor imports.
  - *Fix Applied:* Removed `#[compute]` from `points.glsl`.
- **Theory 4 Confirmed (AABB Culling):** When assigning a buffer directly to `MultiMesh.Buffer`, Godot does not recalculate the AABB. Because `InstanceCount` started at 0, the AABB remained a point at the origin, meaning the point cloud was likely being frustum culled.
  - *Fix Applied:* Added `ExtraCullMargin = 10000.0f` to the `PointsMm` `MultiMeshInstance3D` to bypass culling.
- **Theory 1 / ZeroMQ Thread Crash:** The NetMQ `RequestSocket` crashes if it times out and attempts to `SendFrameEmpty()` again before receiving a response, causing the data collection loop to silently die.
  - *Fix Applied:* Rebuilt the ZeroMQ polling loop to break and recreate the socket upon a timeout, ensuring it gracefully recovers and connects to the server if missed. Added a `try-catch` wrapper with `GD.PrintErr` to catch silent Thread deaths.
- **PCL VoxelGrid Empty Cloud Bug:** Upon inspecting `reconstruction.cpp` logs, `rtabmap::util3d::voxelize(local_cloud, 0.05f)` was reducing 19200 raw points down to 0 points because the internal VoxelGrid filter failed to preserve the `PointXYZRGB` fields or the voxel size stripped all non-dense spatial mappings, resulting in a global Point Cloud output of size 0. Godot naturally wasn't drawing anything because there were 0 points.
  - *Fix Applied:* Implemented a safety fallback in `reconstruction.cpp`. If `voxelize` drops the point count to 0, the map keeps the original raw local cloud (`local_cloud->size() > 0`).

## Next Steps
- Open Godot and run the application to verify that the 18,500+ generated map points finally stream and display locally.
