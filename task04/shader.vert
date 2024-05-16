#version 120

// see the GLSL 1.2 specification:
// https://www.khronos.org/registry/OpenGL/specs/gl/GLSLangSpec.1.20.pdf

uniform bool is_reflection; // variable of the program
varying vec3 normal; // normal vector pass to the rasterizer and fragment shader

void main()
{
    normal = vec3(gl_Normal);// set normal and pass it to fragment shader

    // "gl_Vertex" is the *input* vertex coordinate of triangle.
    // "gl_Vertex" has type of "vec4", which is homogeneious coordinate
    float x0 = gl_Vertex.x/gl_Vertex.w;// x-coord
    float y0 = gl_Vertex.y/gl_Vertex.w;// y-coord
    float z0 = gl_Vertex.z/gl_Vertex.w;// z-coord
    if (is_reflection) {
        vec3 nrm = normalize(vec3(0.4, 0.0, 1.0)); // normal of the mirror
        vec3 org = vec3(-0.3, 0.0, -0.5); // point on the mirror
        // wite code to change the input position (x0,y0,z0).
        // the transformed position (x0, y0, z0) should be drawn as the mirror reflection.
        //
        // make sure the occlusion is correctly computed.
        // the mirror is behind the armadillo, so the reflected image should be behind the armadillo.
        // furthermore, make sure the occlusion is correctly computed for the reflected image.
        vec3 p = vec3(x0, y0, z0);
        vec3 v = p - org;
        float d = dot(v, nrm);
        vec3 p_reflected = p - 2.0 * d * nrm;
        x0 = p_reflected.x;
        y0 = p_reflected.y;
        z0 = p_reflected.z;
        // method1: normalize reflected z-coords to [-1,z_mirror]
        // all z-coords of the reflected armadillo are in [-2,z_mirror], so project [-2,z_mirror] to [-1,z_mirror]
        // To be exact, the minimum z-coord of the reflected armadillo is -1.89498 (reflected vtx2xyz.col(442))
        vec3 p_proj_z = vec3(x0, y0, 0.0);
        float z_mirror = dot(org-p_proj_z, nrm) / nrm.z;
        z0 = -1.0 + (z0 + 2.0) * (z_mirror + 1.0) / (z_mirror + 2.0);
    }
    // method2: normalize all z-coords to [-1,1]
    // all z-coords of two armadillos (real and reflected) are in [-2,2], so project [-2,2] to [-1,1]
    // To be exact, z_max = 0.496365 (vtx2xyz.col(123)), z_min = -1.89498 (reflected vtx2xyz.col(442))
    // const float z_max = 2.0;
    // const float z_min = -2.0;
    // z0 = (z0 - z_min) / (z_max - z_min) * 2.0 - 1.0;
    // do not edit below

    // "gl_Position" is the *output* vertex coordinate in the
    // "canonical view volume (i.e.. [-1,+1]^3)" pass to the rasterizer.
    gl_Position = vec4(x0, y0, -z0, 1);// opengl actually draw a pixel with *maximum* depth. so invert z
}
