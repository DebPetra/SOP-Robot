from head_movement.hdmv_cfg import *

class hdmv_logger:
    def __init__(self, node_logger):
        self.logger = node_logger

    def log_trace(self, msg):
        if HDMV_ENABLE_TRACE:
            self.logger.info(f"[trc] {msg}")

    def log_verbose(self, msg):
        if HDMV_ENABLE_VERBOSE:
            self.logger.info(f"[verbose] {msg}")

    def log_warning(self, msg):
        if HDMV_ENABLE_WARNING:
            self.logger.info(f"[warning] {msg}")



