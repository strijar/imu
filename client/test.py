#!/usr/bin/python3

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner

class Component(ApplicationSession):

    def on_angle(self, i):
        print("Got angles: {}".format(i))

    def onJoin(self, details):
        print("session attached")
        self.subscribe(self.on_angle, u'angle')

    def onDisconnect(self):
        print("disconnected")
        reactor.stop()


if __name__ == '__main__':
    runner = ApplicationRunner(u'ws://192.168.49.94:55555', u'imu')
    runner.run(Component)
