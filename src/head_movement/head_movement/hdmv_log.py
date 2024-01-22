import colorama
from head_movement.hdmv_cfg import *

class hdmv_logger:
    def __init__(self, node_logger):
        self.logger = node_logger
        self.ENABLE_TRACE   = True
        self.ENABLE_VERBOSE = True
        self.ENABLE_WARNING = True
        self.ENABLE_FATAL = True

    def log_trace(self, msg):
        if self.ENABLE_TRACE:
            self.logger.info(f"[trc]{colorama.Style.RESET_ALL} {msg}")

    def log_verbose(self, msg):
        if self.ENABLE_VERBOSE:
            self.logger.info(f"{colorama.Fore.BLUE}[verbose]{colorama.Style.RESET_ALL} {msg}")

    def log_warning(self, msg):
        if self.ENABLE_WARNING:
            self.logger.info(f"{colorama.Fore.YELLOW}[warning]{colorama.Style.RESET_ALL} {msg}")

    def log_fatal(self, msg):
        if self.ENABLE_FATAL:
            self.logger.info(f"{colorama.Fore.RED}[fatal]{colorama.Style.RESET_ALL} {msg}")

