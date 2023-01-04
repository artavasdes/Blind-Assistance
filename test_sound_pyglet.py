import pyglet

pyglet.options['audio'] = ('openal')




player = pyglet.media.Player()

source = pyglet.media.StreamingSource()

MediaLoad = pyglet.media.load(vidPath)
# add this media in the queue
player.queue(MediaLoad)
# play the video
player.play()


pyglet.app.run()
