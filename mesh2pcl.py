import argparse
import logging

import numpy as np

from random import seed
from timeit import default_timer as timer


def parse_arguments():
    """Setup CLI interface
    """
    parser = argparse.ArgumentParser(description="")

    parser.add_argument(
        "-i",
        "--path_input",
        type=str,
        default="hp.jpg",
        help="path to input image to use",
    )

    parser.add_argument(
        "-s", "--rand_seed", type=int, default=-1, help="random seed to use"
    )

    # last line to parse the args
    args = parser.parse_args()
    return args


def setup_logger(logLevel="DEBUG"):
    """Setup logger that outputs to console for the module
    """
    logroot = logging.getLogger("c")
    logroot.propagate = False
    logroot.setLevel(logLevel)

    module_console_handler = logging.StreamHandler()

    #  log_format_module = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    #  log_format_module = "%(name)s - %(levelname)s: %(message)s"
    #  log_format_module = '%(levelname)s: %(message)s'
    #  log_format_module = '%(name)s: %(message)s'
    log_format_module = "%(message)s"

    formatter = logging.Formatter(log_format_module)
    module_console_handler.setFormatter(formatter)

    logroot.addHandler(module_console_handler)

    logging.addLevelName(5, "TRACE")
    # use it like this
    # logroot.log(5, 'Exceedingly verbose debug')

    # example log line
    logg = logging.getLogger(f"c.{__name__}.setup_logger")
    logg.debug(f"Done setting up logger")


def setup_env():
    setup_logger()

    args = parse_arguments()

    # setup seed value
    if args.rand_seed == -1:
        myseed = 1
        myseed = int(timer() * 1e9 % 2 ** 32)
    else:
        myseed = args.rand_seed
    seed(myseed)
    np.random.seed(myseed)

    # build command string to repeat this run
    recap = f"python3 lab03_main.py"
    for a, v in args._get_kwargs():
        if a == "rand_seed":
            recap += f" --rand_seed {myseed}"
        else:
            recap += f" --{a} {v}"

    logmain = logging.getLogger(f"c.{__name__}.setup_env")
    logmain.info(recap)

    return args


def linear_combine(a, b, t):
    """compute the linear combination of two points

    x = t*a + (1-t)*b
    """
    logg = logging.getLogger(f"c.{__name__}.linear_combine")
    #  logg.debug(f"Combining {t} : {a}, {b}")

    assert 0 <= t <= 1, f"t has to be between 0 and 1, got {t}"
    t1 = 1 - t
    x = []
    for i in range(3):
        x.append(a[i] * t + b[i] * t1)
    return tuple(x)


def sample_triangle(vertici, num_samples=10):
    """create points on a triangle

    vertici is a 3-tuple of points XYZ
    num_samples is the number of samples per side, the total number of samples is n^2
    """

    triangle_pc = []

    ns1 = num_samples + 1
    for t1 in range(1, num_samples + 1):
        mid_point = linear_combine(vertici[0], vertici[1], t1 / ns1)
        for t2 in range(1, num_samples + 1):
            center_point = linear_combine(vertici[2], mid_point, t2 / ns1)
            triangle_pc.append(center_point)

    return triangle_pc


def sample_triangle_sparse(vertici, num_samples=10):
    """create points on a triangle, more uniformly distributed

    vertici is a 3-tuple of points XYZ

    num_samples = 5
              row
    v[0]
    '         0
    ''        1
    '''       2
    ''''      3
    '''''     4
    v[1] v[2]
    """
    logg = logging.getLogger(f"c.{__name__}.sample_triangle_sparse")
    logg.debug(f"Sampling triangle {vertici}")

    triangle_pc = []
    for row in range(num_samples):
        t_mp = row / (num_samples - 1)
        logg.debug(f"Row {row}, t_mp {t_mp}")
        mp01 = linear_combine(vertici[0], vertici[1], t_mp)
        mp02 = linear_combine(vertici[0], vertici[2], t_mp)
        logg.debug(f"\tmp01 {mp01} mp02 {mp02}")
        for s in range(num_samples - row):
            if num_samples - row - 1 == 0:
                t_s = 0
            else:
                t_s = s / (num_samples - row - 1)
            inside_point = linear_combine(mp01, mp02, t_s)
            logg.debug(f"\t\ts {s} t_s {t_s:.4f} inside_point {inside_point}")
            triangle_pc.append(inside_point)

    return triangle_pc


def save_pc_to_ASCII(pc, pc_file):
    """save a pointcloud to a file with ASCII format

    TEMPLATE:
    # .PCD v0.7 - Point Cloud Data file format
    VERSION 0.7
    FIELDS x y z
    SIZE 4 4 4
    TYPE F F F
    COUNT 1 1 1
    WIDTH 640
    HEIGHT 480
    VIEWPOINT 0 0 0 1 0 0 0
    POINTS 307200
    DATA ascii
    -1.5575031 -1.167518 2.8730519
    -1.5526056 -1.167501 2.8730099
    -1.5477085 -1.167484 2.8729684
    -1.5428114 -1.1674671 2.8729267
    """

    file_template = "# .PCD v0.7 - Point Cloud Data file format\n"
    file_template += "VERSION 0.7\n"
    file_template += "FIELDS x y z\n"
    file_template += "SIZE 4 4 4\n"
    file_template += "TYPE F F F\n"
    file_template += "COUNT 1 1 1\n"
    file_template += f"WIDTH {len(pc)}\n"
    file_template += "HEIGHT 1\n"
    #  file_template += "VIEWPOINT 0 0 0 1 0 0 0\n"
    file_template += f"POINTS {len(pc)}\n"
    file_template += "DATA ascii\n"
    point_template = "{} {} {}\n"
    for point in pc:
        new_point = point_template.format(*point)
        #  print(new_point)
        file_template += new_point

    with open(pc_file, "w") as fp:
        fp.write(file_template)


def run_mesh2pcl(args):
    """
    """
    logg = logging.getLogger(f"c.{__name__}.run_mesh2pcl")
    logg.debug(f"Transforming")

    triangles = {
        0: (0, 1, 2),
        1: (3, 4, 1),
        2: (5, 6, 4),
        3: (7, 8, 6),
        4: (0, 9, 5),
        5: (9, 10, 8),
        6: (11, 2, 10),
        7: (1, 8, 10),
        8: (0, 3, 1),
        9: (3, 5, 4),
        10: (5, 7, 6),
        11: (7, 9, 8),
        12: (5, 3, 0),
        13: (0, 11, 9),
        14: (9, 7, 5),
        15: (9, 11, 10),
        16: (11, 0, 2),
        17: (10, 2, 1),
        18: (1, 4, 6),
        19: (6, 8, 1),
    }
    vertex = {
        0: (-0.000044, 0.052079, 0.094636),
        1: (0.043257, 0.027079, -0.105364),
        2: (-0.000044, 0.052079, -0.105364),
        3: (0.043257, 0.027079, 0.094636),
        4: (0.043257, -0.022921, -0.105364),
        5: (0.043257, -0.022921, 0.094636),
        6: (-0.000044, -0.047921, -0.105364),
        7: (-0.000044, -0.047921, 0.094636),
        8: (-0.043345, -0.022921, -0.105364),
        9: (-0.043345, -0.022921, 0.094636),
        10: (-0.043345, 0.027079, -0.105364),
        11: (-0.043345, 0.027079, 0.094636),
    }
    #  triangles = {
        #  0: (0, 1, 2),
        #  #  1: (0, 2, 3),
        #  #  2: (5, 6, 4),
    #  }
    #  #  scale = 100
    #  scale = 1
    #  vertex = {
        #  0: tuple(a * scale for a in (0.0, 0.0, 0.0)),
        #  1: tuple(a * scale for a in (0.0, 0.1, 0.0)),
        #  2: tuple(a * scale for a in (0.1, 0.1, 0.0)),
        #  3: tuple(a * scale for a in (0.1, 0.0, 0.0)),
    #  }

    pcl_out = []
    #  num_samples = 100
    #  num_samples = 5
    num_samples = 30
    for t in triangles:
        vertici = tuple(vertex[i] for i in triangles[t])
        logg.debug(f"Triangle {t} vertex {triangles[t]} at {vertici}")
        #  triangle_pc = sample_triangle(vertici, num_samples)
        triangle_pc = sample_triangle_sparse(vertici, num_samples)
        pcl_out.extend(triangle_pc)

    logg.debug(f"Total points {len(pcl_out)}")

    pc_file = "./pcl_data/pcl_test_from_mesh.pcd"
    save_pc_to_ASCII(pcl_out, pc_file)


if __name__ == "__main__":
    args = setup_env()
    run_mesh2pcl(args)
