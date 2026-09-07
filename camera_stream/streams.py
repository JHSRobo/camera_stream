import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

Gst.init(None)

class CameraStream:
    def __init__(self, 
                 name,
                 source, 
                 width, 
                 height, 
                 frame_rate, 
                 host,
                 port):
        self.name = name
        self.source = source
        self.width = width
        self.height = height
        self.frame_rate = frame_rate
        self.host = host
        self.port = port
        
    def start(self):
        self.pipeline.set_state(Gst.State.PLAYING)

    def stop(self):
        self.pipeline.set_state(Gst.State.NULL)

class H264Stream(CameraStream):
    def build_pipeline(self):
        self.pipeline = Gst.Pipeline.new(self.name)

        # Source
        source = Gst.ElementFactory.make("v4l2src", "source")
        source.set_property("device", self.source)

        # Queue
        queue = Gst.ElementFactory.make("queue", "queue")
        queue.set_property("max-size-buffers", 1)
        queue.set_property("max-size-bytes", 0)
        queue.set_property("max-size-time", 0)
        queue.set_property("leaky", 2)

        # Capsfilter
        capsfilter = Gst.ElementFactory.make("capsfilter", "caps")

        caps = Gst.Caps.new_empty()

        structure = Gst.Structure.new_empty("video/x-h264")
        structure.set_value("width", self.width)
        structure.set_value("height", self.height)
        structure.set_value("framerate", Gst.Fraction(self.frame_rate, 1))

        caps.append_structure(structure)

        capsfilter.set_property("caps", caps)

        # Parser
        parser = Gst.ElementFactory.make("h264parse", "parser")

        # Payloader
        payloader = Gst.ElementFactory.make("rtph264pay", "payloader")
        payloader.set_property("pt", 96)
        payloader.set_property("config-interval", 1) # Send SPS and PPS parameters every 1 second

        # Sink
        sink = Gst.ElementFactory.make("udpsink", "sink")
        sink.set_property("host", "192.168.2.1")
        sink.set_property("port", self.port)
        sink.set_property("sync", False)
        sink.set_property("async", False)

        elements = [
            source,
            queue,
            capsfilter,
            parser,
            payloader,
            sink
        ]

        for element in elements:
            self.pipeline.add(element)

        source.link(queue)
        queue.link(capsfilter)
        capsfilter.link(parser)
        parser.link(payloader)
        payloader.link(sink)

class MJPEGStream(CameraStream):
    def build_pipeline(self):
        self.pipeline = Gst.Pipeline.new(self.name)

        # Source
        source = Gst.ElementFactory.make("v4l2src", "source")
        source.set_property("device", self.source)

        # Queue
        queue = Gst.ElementFactory.make("queue", "queue")
        queue.set_property("max-size-buffers", 1) # Buffer only holds one frame at a time; only the most recent frame will occupy the buffer
        queue.set_property("max-size-bytes", 0)
        queue.set_property("max-size-time", 0)
        queue.set_property("leaky", 2) # Leaks old frames when buffer fills up

        # Capsfilter
        capsfilter = Gst.ElementFactory.make("capsfilter", "caps")

        caps = Gst.Caps.new_empty()

        structure = Gst.Structure.new_empty("image/jpeg")
        structure.set_value("width", self.width)
        structure.set_value("height", self.height)
        structure.set_value("framerate", Gst.Fraction(self.frame_rate, 1))

        caps.append_structure(structure)

        capsfilter.set_property("caps", caps)

        # Parser
        parser = Gst.ElementFactory.make("jpegparse", "parser") # See if removing this can create lower latency

        # Payloader
        payloader = Gst.ElementFactory.make("rtpjpegpay", "payloader")

        # Sink
        sink = Gst.ElementFactory.make("udpsink", "sink")
        sink.set_property("host", "192.168.2.1")
        sink.set_property("port", self.port)
        sink.set_property("sync", False) # Send packets immediately after received
        sink.set_property("async", False)

        elements = [
            source,
            queue,
            capsfilter,
            parser,
            payloader,
            sink
        ]

        for element in elements:
            self.pipeline.add(element)

        source.link(queue)
        queue.link(capsfilter)
        capsfilter.link(parser)
        parser.link(payloader)
        payloader.link(sink)
