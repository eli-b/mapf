from collections import defaultdict


class keydefaultdict(defaultdict):
    """Like defaultdict, but the default_factory function is called with the
   requested key as an argument"""
    def __missing__(self, key):
        if self.default_factory is None:
            raise KeyError(key)
        else:
            self[key] = ret = self.default_factory(key)
            return ret