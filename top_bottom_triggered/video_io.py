import datetime, subprocess, numpy as np, time, sys,cv2
from multiprocessing import Process, Queue


def get_number_of_frames(filepath):
    command = 'C:\\ffmpeg\\bin\\ffprobe -v error -select_streams v:0 -show_entries stream=nb_frames -of default=nokey=1:noprint_wrappers=1'
    out = subprocess.Popen(command.split(' ')+[filepath], 
               stdout=subprocess.PIPE, 
               stderr=subprocess.STDOUT)
    stdout,stderr = out.communicate()    
    return int(stdout.decode('utf8').strip('\n'))


def write_frames(filename, frames, threads=6, fps=30, crf=10,
                 pixel_format='gray8', codec='h264', close_pipe=True,
                 pipe=None, slices=24, slicecrc=1, frame_size=None, get_cmd=False):
    """
    Write frames to avi file using the ffv1 lossless encoder
    """

    # we probably want to include a warning about multiples of 32 for videos
    # (then we can use pyav and some speedier tools)

    if not frame_size and type(frames) is np.ndarray:
        frame_size = '{0:d}x{1:d}'.format(frames.shape[2], frames.shape[1])

    command = ['C:\\ffmpeg\\bin\\ffmpeg', #MJ: added after installing binary
               '-y',
               '-loglevel', 'fatal',
               '-framerate', str(fps),
               '-f', 'rawvideo',
               '-s', frame_size,
               '-pix_fmt', pixel_format,
               '-i', '-',
               '-an',
               '-crf',str(crf),
               '-vcodec', codec,
               '-preset', 'ultrafast',
               '-threads', str(threads),
               '-slices', str(slices),
               '-slicecrc', str(slicecrc),
               '-r', str(fps),
               filename]
   
    if get_cmd:
        return command

    if not pipe:
        pipe = subprocess.Popen(
            command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
        
    dtype = ('uint16' if '16' in pixel_format else 'uint8')
    for i in range(frames.shape[0]):
        pipe.stdin.write(frames[i,:,:].astype(dtype).tobytes())

    if close_pipe:
        pipe.stdin.close()
        return None
    else:
        return pipe



def read_frames(filename, frames, threads=6, fps=30,
                pixel_format='gray8', frame_size=(640,576),
                slices=24, slicecrc=1, get_cmd=False):
    """
    Reads in frames from the .mp4/.avi file using a pipe from ffmpeg.
    Args:
        filename (str): filename to get frames from
        frames (list or 1d numpy array): list of frames to grab
        threads (int): number of threads to use for decode
        fps (int): frame rate of camera in Hz
        pixel_format (str): ffmpeg pixel format of data
        frame_size (str): wxh frame size in pixels
        slices (int): number of slices to use for decode
        slicecrc (int): check integrity of slices
    Returns:
        3d numpy array:  frames x h x w
    """

    command = [
        'C:\\ffmpeg\\bin\\ffmpeg', #MJ: added after installing binary
        '-loglevel', 'fatal',
        '-ss', str(datetime.timedelta(seconds=frames[0]/fps)),
        '-i', filename,
        '-vframes', str(len(frames)),
        '-f', 'image2pipe',
        '-s', '{:d}x{:d}'.format(frame_size[0], frame_size[1]),
        '-pix_fmt', pixel_format,
        '-threads', str(threads),
        '-slices', str(slices),
        '-slicecrc', str(slicecrc),
        '-vcodec', 'rawvideo',
        '-'
    ]

    if get_cmd:
        return command
    
    pipe = subprocess.Popen(command, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
    out, err = pipe.communicate()
    if(err):
        print('error', err)
        return None
    
    dtype = ('uint16' if '16' in pixel_format else 'uint8')
    video = np.frombuffer(out, dtype=dtype).reshape((len(frames), frame_size[1], frame_size[0]))
    return video


def get_camera_indexes(serial_numbers):
    serials_to_indexes = {}
    info = subprocess.check_output(['C:\\Program Files\\Azure Kinect SDK v1.4.1\\tools\\k4arecorder.exe', '--list']) #MJ: need full path to k4arecorder.exe
    for l in info.decode('utf-8').split('\n')[:-1]:
        print(l)
        index = int(l.split('\t')[0].split(':')[1])
        serial = l.split('\t')[1].split(':')[1]
        serials_to_indexes[serial] = index
    return {name: serials_to_indexes[sn] for name,sn in serial_numbers.items()}

    

def display_images(display_queue):
    while True: 
        data = display_queue.get() 
        if len(data)==0: 
            cv2.destroyAllWindows()
            break
        else:
            ir = data[0]
            cv2.imshow('ir',ir)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break 


def write_images(image_queue, filename_prefix):
    depth_pipe = None
    ir_pipe = None 
    
    while True: 
        data = image_queue.get() 
        if len(data)==0: 
            depth_pipe.stdin.close()
            ir_pipe.stdin.close()
            break
        else:
            ir,depth = data
            depth_pipe = write_frames(filename_prefix+'.depth.avi', depth.astype(np.uint16)[None,:,:], codec='ffv1', pixel_format='gray16', close_pipe=False, pipe=depth_pipe)
            ir_pipe = write_frames(filename_prefix+'.ir.avi', ir.astype(np.uint16)[None,:,:], codec='ffv1', pixel_format='gray16', close_pipe=False, pipe=ir_pipe)


            
def capture_from_azure(k4a, filename_prefix, recording_length, display_frames=False, display_time=False, externally_triggered=False):
    
    image_queue = Queue()
    write_process = Process(target=write_images, args=(image_queue, filename_prefix))
    write_process.start()
    
    if display_frames: 
        display_queue = Queue()
        display_process = Process(target=display_images, args=(display_queue,))
        display_process.start()
        
    ii = 0
    if externally_triggered:
        print('opening device')
        try:
            k4a.open()
            while not k4a.is_running:
                jack_in, jack_out = k4a.sync_jack_status
                if jack_in: 
                    print('received trigger, starting device')
                    k4a.start()
                else:
                    ii += 1
                    if ii % 1000 == 0: print('Waiting for trigger to start...')
        except:
            if k4a.opened: k4a.close()
    else:            
        k4a.start()
        
    system_timestamps = []
    device_timestamps = []
    start_time = time.time()
    count = 0
    
    try:
        while time.time()-start_time < recording_length:  
            capture = k4a.get_capture()
            if capture.depth is None: 
                print('Dropped frame')
                continue
            
            system_timestamps.append(time.time())
            device_timestamps.append(capture.depth_timestamp_usec)

            depth = capture.depth.astype(np.int16)
            ir = capture.ir.astype(np.uint16)
            image_queue.put((ir,depth))
            
            if display_frames and count % 2 == 0: 
                display_queue.put((ir[::2,::2],))

            if display_time and count % 15 == 0: 
                sys.stdout.write('\rRecorded '+repr(int(time.time()-start_time))+' out of '+repr(recording_length)+' seconds')
            count += 1
            
    except OSError:
        print('Recording stopped early')
        
    finally:
        k4a.stop()
        system_timestamps = np.array(system_timestamps) 
        np.save(filename_prefix+'.system_timestamps.npy',system_timestamps)
        np.save(filename_prefix+'.device_timestamps.npy',device_timestamps)
        print(' - Frame rate = ',len(system_timestamps) / (system_timestamps.max()-system_timestamps.min()))

        image_queue.put(tuple())
        write_process.join()

        if display_frames:
            display_queue.put(tuple())
            display_process.join()
            
