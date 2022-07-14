#!/usr/bin/env python3

from multiprocessing import Pool, RLock, shared_memory
from subprocess import Popen, PIPE, STDOUT
from tqdm import tqdm
import itertools
import os
import re
import sys
import time

import_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(import_path, '../common'))
from constants import PLATFORM_DIRECTORIES, LOGO # noqa: E402


def wait_for_others(progress_bar, position, done_flags):
    """
    tqdm will rearrange the status bars moving the first progress bar that
    finishes to the top of the group.

    However this causes the platforms to rarely be in the same order and makes
    it more difficult to see the difference between new and old failures while
    developing.

    This function waits to close the progress bar object until the ones in
    earlier positions have closed first.

    :param progress_bar: tqdm progress bar.
    :type progress_bar: TQDM object
    :param position: Position of the progress bar, starting with 0 at the top.
    :type position: int
    :param done_flags: Shared memory array of flags to signify a progress bar
                       is done.
    :type done_flags: shared_memory.ShareableList
    """

    # We need to adjust the progress bar finish time to not include waiting.
    wait_start = time.time()

    lock = tqdm.get_lock()

    # First progress bar can always close first.
    if position == 0:
        try:
            lock.acquire()
            done_flags[position] = 1

            # Adjust finish time.
            progress_bar.start_t += time.time() - wait_start
            progress_bar.close()
        except Exception as err:
            raise err
        finally:
            lock.release()

    # Everyone else.
    while 1:
        try:
            lock.acquire()
            next_done_flag = done_flags[position - 1]

            if next_done_flag == 1:
                done_flags[position] = 1

                # Adjust finish time.
                progress_bar.start_t += time.time() - wait_start
                progress_bar.close()
                return

        except Exception as err:
            raise err
        finally:
            lock.release()


def build_platform(position, done_flags, platform):
    """
    This function asynchronously builds the specified platform while updating
    the progress with a progress bar.

    The thread then waits to close until all the platforms before it have
    finished building.

    :param position: The vertical position of the progress bar.
    :type position: int
    :param done_flags: Shared memory flags to track finished builds.
    :type done_flags: shared_memory.ShareableList
    :param platform: Platform name for this thread to build.
    :type platform: str
    """

    text = '{platform}'.format(platform=platform)
    text = text.ljust(10)

    # Run west build and always create a pristine build.
    cmd = 'west build -d build/'
    cmd += platform
    cmd += ' -c -p -b ast1030_evb meta-facebook/'
    cmd += platform

    # Start the build process first.
    p = Popen(cmd, stdout=PIPE, stderr=STDOUT, shell=True)

    # The build progress bar won't start until the first compilation step
    # because we don't know how many steps exist before that point.
    #
    # Create a spinner so user know it's not stuck.
    spinner = itertools.cycle(['-', '/', '|', '\\'])
    last_update = round(time.time() * 1000)

    # Create progress bars with percent completion/
    with tqdm(total=100, position=position, desc=text) as progress:

        # Look for output that start with [step / total_steps].
        status_regex = re.compile(r'^\[(.*)\/(.*)\]')

        for line in p.stdout:
            # Only update progress on on matching output.
            status = status_regex.match(line.decode("utf-8"))
            if status is not None and len(status.groups()) == 2:

                # Reset description on first actual step.
                if progress.n == 0:
                    progress.set_description_str(text)

                # Get current percent.
                percent = ((int(status.group(1)) / int(status.group(2))) * 100)

                # Update with the difference between current and prev.
                progress.update(int(percent) - progress.n)

            # Animate spinner before first step.
            elif progress.n == 0:
                curr_time = round(time.time() * 1000)

                # Update spinner.
                if curr_time - last_update > 100:
                    init_string = text + " starting "
                    init_string += next(spinner)
                    progress.set_description_str(init_string)
                    last_update = curr_time

        # Sometimes the stdout will be empty before the process is actually
        # done. Wait to make sure we can get the return code.
        p.wait()

        if p.returncode == 0:
            progress.colour = 'GREEN'
        else:
            progress.colour = 'RED'

        wait_for_others(progress, position, done_flags)


def build_all(platforms):
    """
    Start threads for each platform to build.

    :param platforms: List of all platform names
    :type platforms: list(str)
    """
    # Shared lock for progress bars.
    tqdm.set_lock(RLock())

    num_platforms = len(platforms)

    # Setup args for all threads.
    position_list = list(range(num_platforms))
    done_flags = shared_memory.ShareableList([0] * num_platforms)
    flags = [done_flags] * num_platforms
    lock = tqdm.get_lock()

    args = reversed(list(zip(position_list, flags, platforms)))

    with Pool(processes=num_platforms, initializer=tqdm.set_lock,
              initargs=(lock,)) as p:
        p.starmap(build_platform, args)
    # cleanup
    done_flags.shm.close()
    done_flags.shm.unlink()


if __name__ == '__main__':
    platforms = list(PLATFORM_DIRECTORIES)
    platforms.sort()

    print(LOGO)

    print("Building " + str(len(platforms)) + " platforms")
    print("Each platforms build directory can be found under: "
          r" /build/$platform_name")
    print("")

    build_all(platforms)
