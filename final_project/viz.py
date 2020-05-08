
import os.path as osp
import glob
from collections import OrderedDict

from tqdm import tqdm
import numpy as np
import cv2
from matplotlib import cm

H = 4.0
W = 4.0
nbins_x = 400
nbins_y = 400
bin_width_x = W/nbins_x
bin_width_y = H/nbins_y

def _get_npz_timestamp(p):
    f = osp.basename(p)
    name = osp.splitext(f)[0]
    timestamp = int(name.split("_")[-1])
    return timestamp


sim_name = "dam_break_2d"
# sim_name = "poiseuille"
npz_file_glob = "{}_output/{}_*.npz".format(sim_name, sim_name)


vw = None

npz_files = glob.glob(npz_file_glob)
npz_files = sorted(npz_files, key=_get_npz_timestamp)

viz_config = OrderedDict(
    p=dict(
        MIN_VALUE=-10000,
        MAX_VALUE=15000,
    ),
    rho=dict(
        MIN_VALUE=900,
        MAX_VALUE=1100,
    ),
    u=dict(
        MIN_VALUE=-10,
        MAX_VALUE=10,
    ),
    v=dict(
        MIN_VALUE=-10,
        MAX_VALUE=10,
    ),
)

# iterate over frames
for frame_idx, npz_file in enumerate(tqdm(npz_files)):
    # if frame_idx > 20:
    #     break
    arrs = np.load(npz_file)  # keys: ['version', 'particles', 'solver_data']


    particle_data = arrs['particles'][()]  # keys: ['fluid', 'boundary']
    fluid_data = particle_data['fluid']  # keys: ['properties', 'constants', 'output_property_arrays', 'arrays']
    fluid_arrays = fluid_data['arrays']  # keys: ['x', 'y', 'z', 'u', 'v', 'w', 'rho', 'm', 'h', 'pid', 'gid', 'tag', 'p']
    # boundary_data = particle_data['boundary']  # keys: ['properties', 'constants', 'output_property_arrays', 'arrays']
    # boundary_arrays = fluid_data['arrays']  # keys: ['x', 'y', 'z', 'u', 'v', 'w', 'rho', 'm', 'h', 'pid', 'gid', 'tag', 'p']
    num_particles = len(fluid_arrays['x'])
    
    # from IPython import embed;embed()
    
    ims = []
    for value_name, value_config in viz_config.items():
        im = np.zeros((nbins_y, nbins_x, 3), dtype=np.uint8)
        MIN_VALUE = value_config['MIN_VALUE']
        MAX_VALUE = value_config['MAX_VALUE']
        for ith in range(num_particles):
            coord_x = int(fluid_arrays['x'][ith] // (bin_width_x))
            coord_y = (nbins_y-1) - int(fluid_arrays['y'][ith] // (bin_width_y))
            if (0 <= coord_x <= (im.shape[1]-1)) and (0 <= coord_y <= (im.shape[0]-1)):
                value = fluid_arrays[value_name][ith]
                value_normalized = (value - MIN_VALUE) / (MAX_VALUE - MIN_VALUE)
                color_idx = int(255*value_normalized)
                color = cm.jet(color_idx)
                color = (np.array(color[2::-1])*255).astype(np.uint8)
                im[coord_y, coord_x, :] = color
        im = cv2.putText(im, value_name, (20, 20), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255 ,255), 1)
        resize_ratio=1.0
        im = cv2.resize(im, (0, 0), fx=resize_ratio, fy=resize_ratio)

        ims.append(im)
    
    margin_width = 5

    h_split = 255*np.ones((ims[0].shape[1], margin_width, 3), dtype=np.uint8)
    v_split = 255*np.ones((margin_width, 2*ims[0].shape[0]+margin_width, 3), dtype=np.uint8)

    im_show = cv2.vconcat([
        cv2.hconcat([ims[0], h_split, ims[1]]),
        v_split,
        cv2.hconcat([ims[2], h_split, ims[3]]),
    ])

    if vw is None:
        output_video = "{}.mp4".format(sim_name)
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        vw = cv2.VideoWriter(
            output_video, fourcc, 10,
            (im_show.shape[1], im_show.shape[0]))
    vw.write(im_show)

if vw is not None:
    vw.release()

# from IPython import embed;embed()
