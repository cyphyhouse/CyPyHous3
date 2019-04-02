from ars.app import Program

class FallingBall(Program):

   def create_sim_objects(self):
      # add_sphere's arguments: radius, center, density
      self.sim.add_sphere(0.5, (1, 10, 1), density=1)

sim_program = FallingBall()
sim_program.start()
sim_program.finalize()
