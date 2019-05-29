import Motion
import numpy as np

quad_m = Motion.Quadcopter()

dest1 = np.array([1., 2., 3.])
dest2 = np.array([4., 5., 6.])

quad_m.start()

quad_m.goTo(dest1)
quad_m.goTo(dest2)