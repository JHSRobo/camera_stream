from abc import ABC, abstractmethod
from pathlib import Path

from data_classes import StreamConfig

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

Gst.init(None)

class CameraStream(ABC):
    def __init__(self, stream_config: StreamConfig):
        self.config = stream_config
        self.pipeline = _build_pipeline()


    # -------------------- Public methods --------------------

    def start(self):
        result = self.pipeline.set_state(Gst.State.PLAYING)

        if result == Gst.StateChangeReturn.FAILURE:
            self.stop()
            raise RuntimeError("Failed to start stream.")

    def stop(self):
        self.pipeline.set_state(Gst.State.NULL)


    # -------------------- Protected methods --------------------
 
    def _build_pipeline(self) -> Gst.Pipeline:
        pipeline = Gst.Pipeline.new(self.config.name)

        if pipeline is None:
            raise RuntimeError(f"Failed to create pipeline: {self.config.name}")

        source = self._create_source()
        queue = self._create_leaky_queue()
        capsfilter = self._create_capsfilter()
        parser = self._create_parser()
        payloader = self._create_payloader()
        sink = self._create_sink()

        elements = [source,
                    queue,
                    capsfilter,
                    parser,
                    payloader,
                    sink]

        try:
            for element in elements:
                pipeline.add(element)

            for upstream, downstream in zip(elements, elements[1:]):
                if not upstream.link(downstream):
                    raise RuntimeError(f"Failed to link {upstream.get_name()} to {downstream.get_name()}")

        except Exception:
            pipeline.set_state(Gst.State.NULL)
            raise

        return pipeline

    @staticmethod
    def _make_element(factory: str, name: str) -> Gst.Element:
        element = Gst.ElementFactory.make(factory, name)

        if element is None:
            raise RuntimeError(f"Failed to create Gstreamer element: {factory}.")

        return element

    def _create_source(self) -> Gst.Element:
        source = _make_element("v4l2src", "source")
        source.set_property("device", self.config.source)

        return source

    def _create_leaky_queue(self) -> Gst.Element:
        queue = _make_element("queue", "queue")
        queue.set_property("max-size-buffers", 1)
        queue.set_property("max-size-bytes", 0)
        queue.set_property("max-size-time", 0)
        queue.set_property("leaky", 2)

        return queue

    @abstractmethod
    def _create_capsfilter(self) -> Gst.Element:
        pass

    @abstractmethod
    def _create_parser(self) -> Gst.Element:
        pass

    @abstractmethod
    def _create_payloader(self) -> Gst.Element:
        pass

    def _create_sink(self):
        sink = _make_element("udpsink", "sink")
        sink.set_property("host", self.config.host)
        sink.set_property("port", self.config.port)
        sink.set_property("sync", False)
        sink.set_property("async", False)

        return sink

class H264Stream(CameraStream):
    def _create_capsfilter(self) -> Gst.Element:
        capsfilter = _make_element("capsfilter", "caps")

        caps = Gst.Caps.new_empty()

        structure = Gst.Structure.new_empty("video/x-h264")
        structure.set_value("width", self.config.width)
        structure.set_value("height", self.config.height)
        structure.set_value("framerate", Gst.Fraction(self.config.frame_rate, 1))

        caps.append_structure(structure)

        capsfilter.set_property("caps", caps)

        return capsfilter

    def _create_parser(self) -> Gst.Element:
        parser = _make_element("h264parse", "parser")

        return parser

    def _create_payloader(self) -> Gst.Element:
        payloader = _make_element("rtph264pay", "payloader")
        payloader.set_property("pt", 96)
        payloader.set_property("config-interval", 1)

        return payloader

class MJPEGStream(CameraStream):
    def _create_capsfilter(self) -> Gst.Element:
        capsfilter = _make_element("capsfilter", "caps")

        caps = Gst.Caps.new_empty()

        structure = Gst.Structure.new_empty("image/jpeg")
        structure.set_value("width", self.width)
        structure.set_value("height", self.height)
        structure.set_value("framerate", Gst.Fraction(self.frame_rate, 1))

        caps.append_structure(structure)

        capsfilter.set_property("caps", caps)

        return capsfilter

    def _create_parser(self) -> Gst.Element:
        parser = _make_element("jpegparse", "parser")

        return parser

    def _create_payloader(self) -> Gst.Element:
        payloader = _make_element("rtpjpegpay", "payloader")

        return payloader
