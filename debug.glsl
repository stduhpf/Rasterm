//this shader is there to debug the voxelSkipLOD() function. And it seems to work fine here, so I am deeply confused

#define DEPTH 6
const int octree_span = 1 << DEPTH;

ivec2 octreeCoordsLod(vec2 p, int lod) {
    return ivec2(float(octree_span) * p) >> lod;
}

vec2 voxelOrigLod(ivec2 ip, int lod) {
    return vec2(ip << lod) / float(octree_span);
}

vec2 skipLOD(vec2 p, vec2 rd, int lod) {
    ivec2 sd = ivec2(sign(rd) * .5 + .5);
    ivec2 ip = octreeCoordsLod(p, lod);
    ivec2 pn = ip + sd;

    vec2 pe = voxelOrigLod(pn, lod);

    vec2 dists = (pe - p) / rd;
    float d = min(dists.x, dists.y);
    // even a very small offset works perfectly there
    d += 1e-6;  
    return p + d * rd;
}

void mainImage(out vec4 fragColor, in vec2 fragCoord) {
    // Normalized pixel coordinates (from 0 to 1)
    vec2 uv = fragCoord / iResolution.xy;

    vec2 p = vec2(.29, .3);
    vec2 d = normalize(iMouse.xy / iResolution.xy - p);

    vec3 color = .25 + vec3(.75) * smoothstep(.0, .005, distance(p, uv));

    // grid (a few pixels off, but no biggie)
    for(int i = DEPTH - 1; i >= 0; i--) {
        vec2 ip = voxelOrigLod(octreeCoordsLod(uv, i), i);
        color *= 1. - .25 * float(min((uv.x - ip.x) * iResolution.y, (uv.y - ip.y) * iResolution.x) < 1.7);
    }

    for(int i = 0; i < 5; i++) {
        p = skipLOD(p, d, max(DEPTH - i - 1, 0));
        color *= .25 + .75 * smoothstep(.0, .005, distance(p, uv));
    }

    // Output to screen
    fragColor = vec4(sqrt(color), 1.0);
}