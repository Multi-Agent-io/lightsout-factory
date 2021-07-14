#!/usr/bin/env python

"""
    Provides classes for quick threading of object methods

    In your class, just add this line to the __init__:
        self.post=Post(self)

    You can now call any object methods with object.post.method(args)
    The Thread object is returned.

"""

from threading import Thread

thread_list = []


class PostThread(Thread):
    """ Object that manages the threaded execution of a given function """

    def __init__(self, func):
        global thread_list

        """ Creates the thread object with the method to be called """
        Thread.__init__(self, )
        self.func = func
        self.isRunning = True
        self.result = None
        self.daemon = True

        thread_list.append([len(thread_list), self])

    def execute(self, *args, **kwargs):
        """ Store the method call arguments and start the thread, returns the thread object to the caller """
        self.args = args
        self.kwargs = kwargs
        self.start()
        return self

    def run(self):
        """ Thread execution, call the function with saved arguments, saves result and change flag at the end """
        self.result = self.func(*self.args, **self.kwargs)
        self.isRunning = False

    def kill(self):
        global thread_list

        if self.is_alive():
            thread_list.remove(self)
            self._Thread__stop()


class Post:
    """ Object that provides threaded calls to its parent object methods """

    post_thread = None
    p_id = 0

    def __init__(self, parent):
        self.parent = parent

    def __getattr__(self, attr):
        """ Find the method asked for in parent object, encapsulate in a PostThread object and send back pointer to execution function"""
        try:
            func = getattr(self.parent, attr)
            self.post_thread = PostThread(func)
            return self.post_thread.execute
        except:
            raise Exception("ERROR: Post call on %s: method %s not found" % (str(self.parent), attr))

    def __kill(self):
        self.post_thread.kill()