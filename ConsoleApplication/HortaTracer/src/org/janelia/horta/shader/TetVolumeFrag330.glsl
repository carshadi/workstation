#version 330

/**
 * Tetrahdedral volume rendering fragment shader.
 */

/* 
 * Licensed under the Janelia Farm Research Campus Software Copyright 1.1
 * 
 * Copyright (c) 2014, Howard Hughes Medical Institute, All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 *     1. Redistributions of source code must retain the above copyright notice, 
 *        this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright 
 *        notice, this list of conditions and the following disclaimer in the 
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the Howard Hughes Medical Institute nor the names 
 *        of its contributors may be used to endorse or promote products derived 
 *        from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, ANY 
 * IMPLIED WARRANTIES OF MERCHANTABILITY, NON-INFRINGEMENT, OR FITNESS FOR A 
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * REASONABLE ROYALTIES; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#extension GL_ARB_shading_language_420pack : enable

layout(binding = 0) uniform sampler3D volumeTexture;

in vec4 barycentricCoord;
in vec3 fragTexCoord;
flat in vec3 cameraPosInTexCoord;
flat in mat4 tetPlanesInTexCoord;

out vec4 fragColor;

// Return ray parameter where ray intersects plane
void clipRayToPlane(
        in vec3 rayStart, in vec3 rayDirection, 
        in vec4 plane,
        inout float begin, // current ray start parameter
        inout float end) // current ray end parameter
{
    vec3 n = plane.xyz;
    float d = plane.w;
    vec3 x0 = rayStart;
    vec3 x1 = rayDirection;
    float intersection = -(dot(x0, n) + d) / dot(n, x1);
    float direction = dot(n, x1);
    if (direction == 0)
        return; // ray is parallel to plane
    if (direction > 0) { // plane normal is along ray direction
        begin = max(begin, intersection);
    }
    else { // plane normal is opposite to ray direction
        end = min(end, intersection);
    }
}

void main() {
    vec4 b = barycentricCoord;
    float f = min(b.x, min(b.y, min(b.z, b.w)));

    if (f < 0) discard; // outside of tetrahedron; should not happen

#ifdef DISPLAY_EDGES_ONLY
    // Display only the edges of the triangle
    float e1 = 0; // b.x + b.y; // first leg of base triangle
    float e2 = 0; // b.x + b.z; // third leg of base triangle
    float e3 = 0; // b.y + b.z; // second leg of base triangle
    float e4 = b.x + b.w;
    float e5 = b.y + b.w;
    float e6 = b.z + b.w;
    float edge_score = max(e1, max(e2, max(e3, max(e4, max(e5, e6)))));
    if (edge_score < 0.95) discard; // hollow out non-edge region
#endif

    vec3 color = textureLod(volumeTexture, fragTexCoord, 5).rgb; // intentionally downsampled
    float opacity = max(color.r, max(color.g, color.b));

    // Ray parameters
    vec3 x0 = cameraPosInTexCoord; // origin
    vec3 x1 = fragTexCoord - x0; // direction

    // Clip near and far ray bounds
    float minRay = 0; // eye position
    float maxRay = 1; // back face fragment location

    // Clip ray bounds to tetrahedral faces
    clipRayToPlane(x0, x1, tetPlanesInTexCoord[0], minRay, maxRay);
    clipRayToPlane(x0, x1, tetPlanesInTexCoord[1], minRay, maxRay);
    clipRayToPlane(x0, x1, tetPlanesInTexCoord[2], minRay, maxRay);
    clipRayToPlane(x0, x1, tetPlanesInTexCoord[3], minRay, maxRay);
    
    vec3 frontTexCoord = x0 + minRay * x1;
    vec3 rearTexCoord = x0 + maxRay * x1;

    fragColor = vec4(

            // reduce max intensity, to keep color channels from saturating to white
            // 0.3 * fragTexCoord.rgb, 1.0 // For debugging texture coordinates
            0.3 * frontTexCoord, 1.0 // For debugging texture coordinates
            // 0.3 * barycentricCoord.rgb, 1.0 // For debugging barycentric coordinates
            // 0.15 * (normalize(x1) + vec3(1, 1, 1)), 1.0 // Direction toward camera
            // 0.3 * cameraPosInTexCoord, 1.0 // Direction toward camera

            // color, 1.0 // For debugging texture image

    );
}