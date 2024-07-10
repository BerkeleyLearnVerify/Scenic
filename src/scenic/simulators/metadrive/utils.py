from metadrive.envs import MetaDriveEnv
from metadrive.manager import BaseManager
from metadrive.component.vehicle.vehicle_type import DefaultVehicle
from metadrive.policy.idm_policy import IDMPolicy
from metadrive.component.traffic_participants.pedestrian import Pedestrian

class DriveManager(BaseManager):
    def __init__(self):
        super(DriveManager, self).__init__()
        self.generated_v = None
        self.ped = None
        self.generate_ts = 0

    def cleanup_physics_bodies(self):
        for obj in self.get_objects():
            print(obj)
            # import pdb; pdb.set_trace()
            # obj.cleanup_physics_body()  # Implement this method as needed

    def reset(self):
        self.cleanup_physics_bodies()
        super().reset()  # Call super method after cleanup
        
    def before_step(self):
        # if self.generated_v:
        #     p = self.get_policy(self.generated_v.id)
        #     self.generated_v.before_step(p.act())
        self.cleanup_physics_bodies()
        pass
        

    def after_step(self):
        # if self.episode_step == self.generate_ts:
        #     self.generated_v = self.spawn_object(DefaultVehicle, 
        #                           vehicle_config=dict(), 
        #                           position=(10, 0), 
        #                           heading=0)
        #     self.add_policy(self.generated_v.id, IDMPolicy, self.generated_v, self.generate_seed())
        #     self.ped = self.spawn_object(
        #         Pedestrian,
        #         name="ped",
        #         position=(50,0),
        #         heading_theta=1,
        #     )
        #     self.ped.set_velocity([1, 0], 1, in_local_frame=True)
        pass
            

            
        # elif self.episode_step == self.recycle_ts:
        #     self.clear_objects([self.generated_v.id])
        #     self.generated_v = None
        # elif self.generated_v:
        #     self.generated_v.after_step()

class DriveEnv(MetaDriveEnv):
    def setup_engine(self):
        super(DriveEnv, self).setup_engine()
        self.engine.register_manager("drive_mgr", DriveManager())