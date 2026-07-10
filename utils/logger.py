"""Preallocated numpy logger, akin to the old MATLAB Logger.m."""
from __future__ import annotations

import numpy as np


class Logger:
    def __init__(self, num_channels, num_steps, channel_names=None):
        self.log = np.full((num_channels, num_steps), np.nan)
        self.time = np.full(num_steps, np.nan)
        self.channel_names = channel_names

    def update(self, data, step, time):
        self.log[:, step] = data
        self.time[step] = time

    def truncate(self):
        """Drop unused preallocated tail columns (where time is still NaN)."""
        valid = ~np.isnan(self.time)
        self.log = self.log[:, valid]
        self.time = self.time[valid]
