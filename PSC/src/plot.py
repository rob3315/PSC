from argparse import ArgumentParser

import numpy
from mayavi import mlab

from plyfile import PlyData


def main():
    f=PlyData.read('/Users/remi/Downloads/pc_really_cool.ply')


    plot(f)
    print(2)
    mlab.show()


def plot(ply):
    '''
    Plot vertices and triangles from a PlyData instance. Assumptions:
        `ply' has a 'vertex' element with 'x', 'y', and 'z'
            properties;
        `ply' has a 'face' element with an integral list property
            'vertex_indices', all of whose elements have length 3.
    '''
    vertex = ply['vertex']
    print(1)
    (x, y, z) = (vertex[t] for t in ('x', 'y', 'z'))

    mlab.points3d(x, y, z, color=(1, 1, 1), mode='point')

    if 'face' in ply:
        tri_idx = ply['face']['vertex_indices']
        idx_dtype = tri_idx[0].dtype

        triangles = numpy.fromiter(tri_idx, [('data', idx_dtype, (3,))],
                                   count=len(tri_idx))['data']

        mlab.triangular_mesh(x, y, z, triangles,
                             color=(1, 0, 0.4), opacity=0.5)


main()