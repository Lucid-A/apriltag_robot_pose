#! /usr/bin/env python
from math import pi, cos, sin

standalone_tag_edge_len = 0.0385 # m
standalone_tags_id = set([i for i in range(18, 32)])

grid_tags_bundle_name = "map"
grid_tags_bundle_edge_len = 0.0385 # m
grid_tags_bundle_id = set([i for i in range(0,18)])
grid_tags_bundle_shape = (3,6) # (x,y) (c,r)

def get_repeated_id(st1 : set, st2 : set) -> set:
    return st1 & st2

def get_identical_id(main : set, common : set) -> set:
    return main ^ common

if __name__ == "__main__":
    common = standalone_tags_id & grid_tags_bundle_id
    print("The following ids are repeated: {common}")
    print("These repeated ids will be excluded in standalone tags and be included in tag bundle")

    # standalone tags
    standalone_tags_id = standalone_tags_id ^ common
    width = len(str(max(standalone_tags_id)))
    standalone_tags = "standalone_tags:\n  [\n"
    for id in (standalone_tags_id):
        standalone_tags = standalone_tags + f"{'':4s}{{id: {id:{width}d}, size: {standalone_tag_edge_len}, name: tag_{id:<{width}d}}},\n"
    standalone_tags = standalone_tags[:-2]
    standalone_tags = standalone_tags + "\n  ]"
    print(standalone_tags)
    print()

    # tag_bundle
    grid_tags_bundle = f"{'':4s}{{\n{'':6s}name: \"{grid_tags_bundle_name}\",\n{'':6s}layout:\n{'':6s}[\n"
    for id in (grid_tags_bundle_id):
        x, y, z = id // grid_tags_bundle_shape[1], id % grid_tags_bundle_shape[1], 0
        qw, qx, qy, qz = cos(-pi/4),0,0,sin(-pi/4)
        grid_tags_bundle = grid_tags_bundle + f"{'':8s}{{\n{'':10s}id: {id:<6d}, size: {grid_tags_bundle_edge_len},\n"
        grid_tags_bundle = grid_tags_bundle + f"{'':10s} x: {x:.6f},  y: {y:.6f},  z: {0:.6f},\n"
        grid_tags_bundle = grid_tags_bundle + f"{'':10s}qw: {qw:.6f}, qx: {qx:.6f}, qy: {qy:.6f}, qz: {qz:.6f}\n{'':8s}}},\n"
        
    grid_tags_bundle = grid_tags_bundle[:-2]
    grid_tags_bundle = grid_tags_bundle + f"\n{'':6s}]\n{'':4s}}}"
    print(grid_tags_bundle)

