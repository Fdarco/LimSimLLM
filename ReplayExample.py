from rich import print

from simModel.egoTracking.replay import ReplayModel

import argparse
parser = argparse.ArgumentParser(description='input dir')
parser.add_argument('--dir', type=str, help='replay dir', default='', required=False)

args = parser.parse_args()

dataBase = args.dir + 'egoTrackingTest.db'

rmodel = ReplayModel(dataBase=dataBase,
                     startFrame=0)

while not rmodel.tpEnd:
    rmodel.moveStep()

rmodel.gui.destroy()
