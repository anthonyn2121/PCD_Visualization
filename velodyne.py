import numpy as np 
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D
import glob 
import pykitti
import parseTrackletXML as xmlParser
import argparse

basedir = '/media/anthony/My Book/Khang School/Independent Projects/lidar_visualize/'
date = '2011_09_26'
drive = '0001'
xmlFile = '2011_09_26/2011_09_26_drive_0001_sync/tracklet_labels.xml'


def load_data(basedir, date, drive, calibrated=False, frame_range=None): 
    dataset = pykitti.raw(basedir, date, drive)
    print('Drive: {}'.format(dataset.drive))
    print('Frame Range: {}'.format(dataset.frames))    
    
    return dataset

def load_tracklets(n_frames, xml_path):   
    tracklets = xmlParser.parseXML(xml_path)

    frame_tracklets = {}
    frame_tracklets_types = {}
    for i in range(n_frames):
        frame_tracklets[i] = []
        frame_tracklets_types[i] = []

    # loop over tracklets
    for i, tracklet in enumerate(tracklets):
        # this part is inspired by kitti object development kit matlab code: computeBox3D
        h, w, l = tracklet.size
        # in velodyne coordinates around zero point and without orientation yet
        trackletBox = np.array([
            [-l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2],
            [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2],
            [0.0, 0.0, 0.0, 0.0, h, h, h, h]
        ])
        # loop over all data in tracklet
        for translation, rotation, state, occlusion, truncation, amtOcclusion, amtBorders, absoluteFrameNumber in tracklet:
            # determine if object is in the image; otherwise continue
            if truncation not in (xmlParser.TRUNC_IN_IMAGE, xmlParser.TRUNC_TRUNCATED):
                continue
            # re-create 3D bounding box in velodyne coordinate system
            yaw = rotation[2]  # other rotations are supposedly 0
            assert np.abs(rotation[:2]).sum() == 0, 'object rotations other than yaw given!'
            rotMat = np.array([
                [np.cos(yaw), -np.sin(yaw), 0.0],
                [np.sin(yaw), np.cos(yaw), 0.0],
                [0.0, 0.0, 1.0]
            ])
            cornerPosInVelo = np.dot(rotMat, trackletBox) + np.tile(translation, (8, 1)).T
            frame_tracklets[absoluteFrameNumber] = frame_tracklets[absoluteFrameNumber] + [cornerPosInVelo]
            frame_tracklets_types[absoluteFrameNumber] = frame_tracklets_types[absoluteFrameNumber] + [
                tracklet.objectType]

    return (frame_tracklets, frame_tracklets_types)

def draw_box(pyplot_axis, vertices, axes=[0, 1, 2], color='black'):
    """
    Draws a bounding 3D box in a pyplot axis.
    
    Parameters
    ----------
    pyplot_axis : Pyplot axis to draw in.
    vertices    : Array 8 box vertices containing x, y, z coordinates.
    axes        : Axes to use. Defaults to `[0, 1, 2]`, e.g. x, y and z axes.
    color       : Drawing color. Defaults to `black`.
    """
    vertices = vertices[axes, :]
    connections = [
        [0, 1], [1, 2], [2, 3], [3, 0],  # Lower plane parallel to Z=0 plane
        [4, 5], [5, 6], [6, 7], [7, 4],  # Upper plane parallel to Z=0 plane
        [0, 4], [1, 5], [2, 6], [3, 7]  # Connections between upper and lower planes
    ]
    for connection in connections:
        pyplot_axis.plot(*vertices[:, connection], c=color, lw=0.5)

def draw_pcd(dataset, frame, axes=[0,1,2], points = 0.2, xlim=None, ylim=None, zlim=None): 

    ## Get velodyne data 
    dataset_velo = list(dataset.velo)
    step_size = int(1/points)
    velo_range = range(0, dataset_velo[frame].shape[0], step_size)
    velo_frame = dataset_velo[frame][velo_range, :] # point cloud data for specific frame 

    
    point_size = 0.01 * (1//points)
    fig = plt.figure()
    if len(axes)>2: 
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(*np.transpose(velo_frame[:,axes]), s = point_size, c=velo_frame[:,3], cmap='gray')
        ax.set_xlim3d(*axes_limit[0])
        ax.set_ylim3d(*axes_limit[1])
        ax.set_zlim3d(*axes_limit[2])
    else: 
        ax = fig.add_subplot(111)
        ax.scatter(*np.transpose(velo_frame[:, axes]), s = point_size, c=velo_frame[:,3], cmap='gray')
        ax.set_xlim(*axes_limit[axes[0]])
        ax.set_ylim(*axes_limit[axes[1]])
    
    for track_boxes, track_types in zip(tracklet_rects[frame], tracklet_types[frame]): 
        # print('type: ',track_types)
        # print('HERE!!!!!!: ', Colors[track_types])
        draw_box(ax, track_boxes, axes=axes, color=Colors[track_types])
    


if __name__ == "__main__": 
    ## Get necessary arguments for visualization 
    parser = argparse.ArgumentParser(description='LiDAR visualization from the KITTI open dataset. Specify the basedir, date, drive, and XML Tracklet file')
    parser.add_argument('--basedir', type=str, default=basedir, help='Your dataset directory') 
    parser.add_argument('--date', type=str, default=date, help='Date of drive ex:2011_09_26')
    parser.add_argument('--drive', type=str, default=drive, help='Drive number ex:0001, 0014, etc')
    parser.add_argument('--xmlFile', type=str, default='2011_09_26/2011_09_26_drive_0001_sync/tracklet_labels.xml', help='Path to xml tracklet file')
    parser.add_argument('--view', type=str, default='XYZ', help='Data to view ex: XYZ, XY, YZ, XZ')
    # parser.add_argument('--lim', type=list, default=[[-20,80], [-20,20], [-3,10]], help='A list of lists defining axes limits of a pyplot graph [[Xmin, Xmax],[Ymin, Ymax], [Zmin, Zmax]]')
    parser.add_argument('--xlim', nargs='+', type=int, default=[-20,80], help='X-axis limits on pyplot graph ex: Xmin Xmax')
    parser.add_argument('--ylim', nargs='+', type=int, default=[-20,20], help='Y-axis limits on pyplot graph ex: Ymin Ymax')
    parser.add_argument('--zlim', nargs='+', type=int, default=[-2,10], help='Z-axis limits on pyplot graph ex: Zmin Zmax')
    parser.add_argument('--frame', type=int, default=10, help='Frame number in camera feed. Argument is an int')
    args = parser.parse_args()  
    
    # axes_limit = [[-20,80], [-20,20], [-3,10]]

    basedir = args.basedir 
    date = args.date 
    drive = args.drive 
    xmlFile = args.xmlFile
    axes_limit = [args.xlim, args.ylim, args.zlim] 
    print(args.xlim)
    print(type(args.xlim))
    if args.view == 'XYZ': 
        axes = [0,1,2]
    elif args.view == 'XY': 
        axes = [0,1]
    elif args.view == 'XZ': 
        axes = [0,2] 
    elif args.view == 'YZ': 
        axes = [1,2]

    Colors = {
    'Car': 'b',
    'Tram': 'r',
    'Cyclist': 'g',
    'Van': 'c',
    'Truck': 'm',
    'Pedestrian': 'y',
    'Sitter': 'k'
    }

    ds = load_data(basedir, date, drive)
    num_frames = len(list(ds.velo))
    tracklet_rects, tracklet_types = load_tracklets(num_frames, xmlFile)
    draw_pcd(dataset=ds, frame=args.frame, axes=axes)

    # for i in range(num_frames): 
    #     draw_pcd(dataset=ds, frame=i, axes=axes)
    #     plt.savefig('2011_09_26/2011_09_26_drive_0001_sync/RESULTS/{}.png'.format(i))
    plt.show()