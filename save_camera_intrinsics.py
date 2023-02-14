from pyk4a import *
import json, time, sys


def save_intrinsics(prefix, bottom_device_id=1, top_device_id=0):    
        
    k4a_top = PyK4A(Config(color_resolution=ColorResolution.RES_720P,
                          depth_mode=DepthMode.NFOV_UNBINNED,
                          synchronized_images_only=False,
                          wired_sync_mode=WiredSyncMode.MASTER), device_id=top_device_id)

    k4a_bottom = PyK4A(Config(color_resolution=ColorResolution.OFF,
                              depth_mode=DepthMode.NFOV_UNBINNED,
                              synchronized_images_only=False,
                              wired_sync_mode=WiredSyncMode.SUBORDINATE,
                              subordinate_delay_off_master_usec=640), device_id=bottom_device_id)
    
    print(k4a_top._device_id)
    k4a_top.start()
    k4a_bottom.start()
    time.sleep(1)
    k4a_top.save_calibration_json(prefix+'.top.json')
    k4a_bottom.save_calibration_json(prefix+'.bottom.json')
    k4a_bottom.stop()
    k4a_top.stop()
    
    

if __name__ == "__main__":
    print(sys.argv)
    prefix = sys.argv[1]
    top_device_id = int(sys.argv[2])
    bottom_device_id = int(sys.argv[3])
    print(f'Top ID: {top_device_id} (master), bottom ID: {bottom_device_id} (subordinate)')
    save_intrinsics(prefix, bottom_device_id=bottom_device_id, top_device_id=top_device_id)
    print('Saved intrinsics to `{}.top.json` and `{}.bottom.json`'.format(prefix,prefix))