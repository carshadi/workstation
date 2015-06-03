#version 430

// vertex to fragment shader io
in vec3 N;
in vec3 I;
in vec4 Cs;

// globals
//uniform float edgefalloff;
//uniform float intensity;
//uniform float ambient;

layout (early_fragment_tests) in;
layout (binding = 0, offset = 0) uniform atomic_uint index_counter;
layout (binding = 0, rgba32ui) uniform coherent uimageBuffer list_buffer;
layout (binding = 1, r32ui) uniform coherent uimage2D head_pointer_image;

//out vec4 debugColor;

// entry point
void main()
{
    // Actual fragment shading step
    float edgefalloff=1.0;
    float intensity=0.5;
    float ambient=0.01;

    float opac = dot(normalize(-N), normalize(-I));
    opac = abs(opac);
    opac = ambient + intensity*(1.0-pow(opac, edgefalloff));
    vec4 color =  opac * Cs;
    color.a = opac;

    //vec4 debugColor = vec4(1.0, 1.0, 1.0, 0.0) * opac;
    //debugColor.a = opac;

    // Update head image and linked list

    uint new_index = atomicCounterIncrement(index_counter);

    uint old_head = imageAtomicExchange(head_pointer_image, ivec2(gl_FragCoord.xy), new_index);

    uvec4 item;

    item.x = old_head;

    //item.y = packUnorm4x8(debugColor);
    item.y = packUnorm4x8(color);

    item.z = floatBitsToUint(gl_FragCoord.z);

    item.w = 0;

    int iIndex=int(new_index);

    imageStore(list_buffer, iIndex, item);

}