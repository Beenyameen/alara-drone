#version 450

layout(local_size_x = 256, local_size_y = 1, local_size_z = 1) in;

layout(set = 0, binding = 0, std430) restrict readonly buffer InputBuffer {
    float data[];
} input_buf;

layout(set = 0, binding = 1, std430) restrict writeonly buffer OutputBuffer {
    float data[];
} output_buf;

layout(push_constant, std430) uniform Params {
    uint point_count;
} params;

void main() {
    uint idx = gl_GlobalInvocationID.x;
    if (idx >= params.point_count) {
        return;
    }

    uint in_offset = idx * 6;
    float px = input_buf.data[in_offset];
    float py = input_buf.data[in_offset + 1];
    float pz = input_buf.data[in_offset + 2];
    float r = input_buf.data[in_offset + 3];
    float g = input_buf.data[in_offset + 4];
    float b = input_buf.data[in_offset + 5];

    uint out_offset = idx * 16;
    
    // Transform3D in Godot 4 MultiMesh is strictly 3 rows of 4 floats (Basis X, Y, Z, Origin) followed by 4 floats of Color.
    // Floats 0-3: Basis.X.x, Basis.Y.x, Basis.Z.x, Origin.x
    // Floats 4-7: Basis.X.y, Basis.Y.y, Basis.Z.y, Origin.y
    // Floats 8-11: Basis.X.z, Basis.Y.z, Basis.Z.z, Origin.z
    // Floats 12-15: Color.r, Color.g, Color.b, Color.a

    // Row 0
    output_buf.data[out_offset + 0] = 1.0; // Basis.X.x
    output_buf.data[out_offset + 1] = 0.0; // Basis.Y.x
    output_buf.data[out_offset + 2] = 0.0; // Basis.Z.x
    output_buf.data[out_offset + 3] = px;  // Origin.x

    // Row 1
    output_buf.data[out_offset + 4] = 0.0; // Basis.X.y
    output_buf.data[out_offset + 5] = 1.0; // Basis.Y.y
    output_buf.data[out_offset + 6] = 0.0; // Basis.Z.y
    output_buf.data[out_offset + 7] = -py;  // Origin.y

    // Row 2
    output_buf.data[out_offset + 8] = 0.0; // Basis.X.z
    output_buf.data[out_offset + 9] = 0.0; // Basis.Y.z
    output_buf.data[out_offset + 10] = 1.0; // Basis.Z.z
    output_buf.data[out_offset + 11] = -pz;  // Origin.z

    // Color
    output_buf.data[out_offset + 12] = r;
    output_buf.data[out_offset + 13] = g;
    output_buf.data[out_offset + 14] = b;
    output_buf.data[out_offset + 15] = 1.0;}