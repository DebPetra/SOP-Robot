import colorama
from head_movement.hdmv_cfg import *

class hdmv_logger:
    def __init__(self, node_logger):
        self.logger = node_logger

    def log_trace(self, msg):
        if HDMV_ENABLE_TRACE:
            self.logger.info(f"[trc]{colorama.Style.RESET_ALL} {msg}")

    def log_verbose(self, msg):
        if HDMV_ENABLE_VERBOSE:
            self.logger.info(f"{colorama.Fore.BLUE}[verbose]{colorama.Style.RESET_ALL} {msg}")

    def log_warning(self, msg):
        if HDMV_ENABLE_WARNING:
            self.logger.info(f"{colorama.Fore.YELLOW}[warning]{colorama.Style.RESET_ALL} {msg}")

    def log_fatal(self, msg):
        if HDMV_ENABLE_WARNING:
            self.logger.info(f"{colorama.Fore.RED}[fatal]{colorama.Style.RESET_ALL} {msg}")

