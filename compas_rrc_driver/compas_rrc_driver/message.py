from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import time

__all__ = [
    'Message'
]


class Message(object):
    def __init__(self, instruction, sequence_id=None, feedback=None, feedback_id=0, exec_level=0, feedback_level=0, string_values=None, float_values=None):
        # Header fields
        ticks = time.time()
        self.sec = int(ticks)
        self.nsec = int((ticks - int(ticks)) * 1000)

        # Payload fields
        self.sequence_id = sequence_id
        self.exec_level = exec_level
        self.instruction = instruction
        self.feedback_level = feedback_level
        self.feedback = feedback
        self.feedback_id = feedback_id
        self.string_values = string_values or []
        self.float_values = float_values or []

    @property
    def key(self):
        """Message keys uniquely identify a specific message."""
        return 'msg:{}'.format(self.sequence_id)

    @classmethod
    def from_ros_message(cls, ros_message):
        return cls(ros_message.instruction,
                   sequence_id=ros_message.sequence_id,
                   exec_level=ros_message.exec_level,
                   feedback_level=ros_message.feedback_level,
                   feedback=ros_message.feedback,
                   feedback_id=ros_message.feedback_id,
                   string_values=ros_message.string_values,
                   float_values=ros_message.float_values)

    def to_ros_message(self, ros_cls):
        return ros_cls(instruction=self.instruction,
                       sequence_id=self.sequence_id,
                       exec_level=self.exec_level,
                       feedback_level=self.feedback_level,
                       feedback=self.feedback,
                       feedback_id=self.feedback_id,
                       string_values=self.string_values,
                       float_values=self.float_values)

    @classmethod
    def from_data(cls, data):
        instruction = data['instruction'].encode('ascii')
        feedback = data['feedback'].encode('ascii') if 'feedback' in data else None
        feedback_id = int(data['feedback_id']) if 'feedback_id' in data else 0

        exec_level = 0
        if 'exec_level' in data:
            exec_level = int(data['exec_level'])

        feedback_level = 0
        if 'feedback_level' in data:
            feedback_level = int(data['feedback_level'])

        string_values = data['string_values'] if 'string_values' in data else None
        float_values = data['float_values'] if 'float_values' in data else None

        return cls(instruction,
                   sequence_id=None,
                   exec_level=exec_level,
                   feedback_level=feedback_level,
                   feedback=feedback,
                   feedback_id=feedback_id,
                   string_values=string_values,
                   float_values=float_values)

    def to_data(self):
        return {
            'key': self.key,
            'instruction': self.instruction,
            'sequence_id': self.sequence_id,
            'exec_level': self.exec_level,
            'feedback_level': self.feedback_level,
            'feedback': self.feedback,
            'feedback_id': self.feedback_id,
            'string_values': self.string_values,
            'float_values': self.float_values,
        }

    def __hash__(self):
        assert self.sequence_id is not None
        return hash(int(self.sequence_id))
