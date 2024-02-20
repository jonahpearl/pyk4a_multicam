import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import time
from multiprocessing import Process, Queue, Event
from queue import Empty


class BlitManager:

    def __init__(self, canvas, animated_artists=()):
        """
        Parameters
        ----------
        canvas : FigureCanvasAgg
            The canvas to work with, this only works for sub-classes of the Agg
            canvas which have the `~FigureCanvasAgg.copy_from_bbox` and
            `~FigureCanvasAgg.restore_region` methods.

        animated_artists : Iterable[Artist]
            List of the artists to manage
        """
        self.canvas = canvas
        self._bg = None
        self._artists = []

        for a in animated_artists:
            self.add_artist(a)
        # grab the background on every draw
        self.cid = canvas.mpl_connect("draw_event", self.on_draw)

    def on_draw(self, event):
        """Callback to register with 'draw_event'."""
        cv = self.canvas
        if event is not None:
            if event.canvas != cv:
                raise RuntimeError
        self._bg = cv.copy_from_bbox(cv.figure.bbox)
        self._draw_animated()

    def add_artist(self, art):
        """
        Add an artist to be managed.

        Parameters
        ----------
        art : Artist

            The artist to be added.  Will be set to 'animated' (just
            to be safe).  *art* must be in the figure associated with
            the canvas this class is managing.

        """
        if art.figure != self.canvas.figure:
            raise RuntimeError
        art.set_animated(True)
        self._artists.append(art)

    def _draw_animated(self):
        """Draw all of the animated artists."""
        fig = self.canvas.figure
        for a in self._artists:
            fig.draw_artist(a)

    def update(self):
        """Update the screen with animated artists."""
        cv = self.canvas
        fig = cv.figure
        # paranoia in case we missed the draw event,
        if self._bg is None:
            self.on_draw(None)
        else:
            # restore the background
            cv.restore_region(self._bg)
            # draw all of the animated artists
            self._draw_animated()
            # update the GUI state
            cv.blit(fig.bbox)
        # let the GUI event loop process anything it has to do
        cv.flush_events()


class ThermistorAnimator():
    """Animates the thermistor signal (or any other signal you pass it) with mpl blitting
    """
    def __init__(
        self,
        n_samples,
        fs,
        signal_header_name='therm',
        signal_initial_yvals=None,
        show_opto=False,
        opto_header_name=None,
        q_downsample=15,
    ):
        self.nsamp = n_samples
        self.signal_header_name = signal_header_name

        self.show_opto = show_opto
        self.opto_header_name = 'led' if opto_header_name is None else opto_header_name

        self.current_val = 0
        self.data = np.zeros((n_samples,), dtype='float')
        self.data_head_idx = 0  # to trace out data like an o-scope

        self.queue = Queue()
        self.animator_exit_event = Event()
        self.animate_process = main_from_ipynb(
            self.queue,
            int(fs/q_downsample),
            self.animator_exit_event,
            n_samples=n_samples,
            ylims=signal_initial_yvals
        )
        self.q_downsample_counter = 0
        self.q_downsample = 15

    def extract_indices_from_header(self, header):
        if self.signal_header_name == 'therm':
            self.therm_idx = [i for i, val in enumerate(header.split(',')) if (val == 'therm' or val == 'thermistor')][0]  # allow therm or thermistor for ease of cross-use
        else:
            self.therm_idx = [i for i, val in enumerate(header.split(',')) if val == self.signal_header_name][0]
        if self.show_opto:
            self.opto_idx = [i for i, val in enumerate(header.split(',')) if val == self.opto_header_name][0]
        self.led_idxs = [i for i, val in enumerate(header.split(',')) if 'led' in val]
        # print(self.signal_header_name, self.therm_idx, self.led_idxs)  # DEBUG: ok

    def start(self):
        if not self.animate_process.is_alive():
            self.animate_process.start()

    def update(self, line):

        # Extract data from correct index in line
        self.current_val = np.array(line.split(',')[self.therm_idx], dtype='float')

        # Update vector
        self.data[self.data_head_idx] = self.current_val

        # Extract opto val if using
        if self.show_opto:
            opto_val = np.array(line.split(',')[self.opto_idx], dtype='float')

            # TODO
            # Show inhales and exhales in super janky way
#                 if np.array(line.split(',')[8], dtype='int') == 1:
#                     therm_data[data_head_idx] = 1000
#                 elif np.array(line.split(',')[9], dtype='int') == 1:
#                     therm_data[data_head_idx] = 0

        # Delete the oldest data to make it o-scope-like
        self.data[(self.data_head_idx + int(self.nsamp/10)) % self.nsamp] = np.nan
        self.data_head_idx += 1
        self.data_head_idx = self.data_head_idx % self.nsamp

        # Increment downsample counter
        self.q_downsample_counter += 1
        self.q_downsample_counter = self.q_downsample_counter % self.q_downsample

        # Decide if sending to animator
        if self.q_downsample_counter == 0:
            sync_tup = tuple([line.split(',')[idx] for idx in self.led_idxs])
            if len(sync_tup) == 0:
                sync_tup = (-1)
            if self.show_opto:
                self.queue.put((self.data, sync_tup, opto_val))
            else:
                q_tup = (self.data, sync_tup, 0)
                self.queue.put(q_tup)

    def close(self):
        self.animator_exit_event.set()
        self.queue.close()
        self.queue.cancel_join_thread()


def animated_plot_process(thermistor_queue, fs, exit_event, n_samples=100, ylims=(0, 1023)):
    matplotlib.use('Qt5Agg')

    # prep data vector
    frame_num = 0
    data = np.zeros((n_samples,))
    queue_size_readable = True

    # make a new figure
    fig, ax = plt.subplots()
    # add a line
    (ln,) = ax.plot(np.arange(n_samples), data, animated=True)
    ax.set_xlim((0, n_samples))
    ax.set_ylim(ylims)
    tic = time.time()

    # add text annotation
    fr_number = ax.annotate(
        "0",
        (0, 1),
        xycoords="axes fraction",
        xytext=(10, -10),
        textcoords="offset points",
        ha="left",
        va="top",
        animated=True,
    )
    bm = BlitManager(fig.canvas, [ln, fr_number])
    # make sure our window is on the screen and drawn
    plt.show(block=False)
    plt.pause(1)

    while not exit_event.is_set():

        # Read data from the queue, without blocking.
        # Will raise the "empty" exception if it's empty, or the ValueError exception if it's closed.
        try:
            data = thermistor_queue.get(block=False)  # tuple of (therm yvals, sync led state, opto), or empty if end
        except Empty:
            data = ()
            pass
        except ValueError:
            data = ()
            pass

        if len(data) == 0:
            pass
        else:
            ln.set_ydata(data[0])
            if (time.time() - tic) > 1:
                time_txt = f"approx time (sec): {time.time() - tic:.2f}"
            led_txt = f"sync: {data[1]}"
            if queue_size_readable:
                try:
                    queue_txt = f"queue sz: {thermistor_queue.qsize()}"
                except NotImplementedError:
                    queue_txt = "queue sz: (not implemented)"
                queue_size_readable = False
            fr_number.set_text("    ".join([time_txt, led_txt, queue_txt]))
            if data[2]:
                ln.set_color('C1')
            else:
                ln.set_color('C0')
            bm.update()

    plt.close(fig)


def main_from_ipynb(data_queue, fs, exit_event, **kwargs):
    """Open a blitting animate process from a jupyter notebook
    """
    animate_process = Process(target=animated_plot_process, args=(data_queue, fs, exit_event), kwargs=kwargs)
    return animate_process


def main():
    """Testing function.
    """
    n_samples = 400
    data_queue = Queue()
    animate_process = Process(target=animated_plot_process, args=(data_queue, n_samples))
    animate_process.start()

    for _ in range(100):
        data = np.random.random((n_samples,))
        t1 = time.time()
        data_queue.put(data)
        print(time.time() - t1)  # report time to pickle this np array
    data_queue.put(tuple())
    animate_process.join()
    print(animate_process.exitcode)
    print('finished animate process')
