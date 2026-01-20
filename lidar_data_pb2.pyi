from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
OK: ProximityStatus
STOP: ProximityStatus
WARNING: ProximityStatus

class Amalgames(_message.Message):
    __slots__ = ["size", "x", "y"]
    SIZE_FIELD_NUMBER: _ClassVar[int]
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    size: _containers.RepeatedScalarFieldContainer[float]
    x: _containers.RepeatedScalarFieldContainer[float]
    y: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, x: _Optional[_Iterable[float]] = ..., y: _Optional[_Iterable[float]] = ..., size: _Optional[_Iterable[float]] = ...) -> None: ...

class Balises(_message.Message):
    __slots__ = ["index", "x", "y"]
    INDEX_FIELD_NUMBER: _ClassVar[int]
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    index: _containers.RepeatedScalarFieldContainer[int]
    x: _containers.RepeatedScalarFieldContainer[float]
    y: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, index: _Optional[_Iterable[int]] = ..., x: _Optional[_Iterable[float]] = ..., y: _Optional[_Iterable[float]] = ...) -> None: ...

class Lidar(_message.Message):
    __slots__ = ["angle_increment", "angles", "distances", "nb_pts", "quality"]
    ANGLES_FIELD_NUMBER: _ClassVar[int]
    ANGLE_INCREMENT_FIELD_NUMBER: _ClassVar[int]
    DISTANCES_FIELD_NUMBER: _ClassVar[int]
    NB_PTS_FIELD_NUMBER: _ClassVar[int]
    QUALITY_FIELD_NUMBER: _ClassVar[int]
    angle_increment: float
    angles: _containers.RepeatedScalarFieldContainer[float]
    distances: _containers.RepeatedScalarFieldContainer[float]
    nb_pts: int
    quality: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, nb_pts: _Optional[int] = ..., angle_increment: _Optional[float] = ..., angles: _Optional[_Iterable[float]] = ..., distances: _Optional[_Iterable[float]] = ..., quality: _Optional[_Iterable[float]] = ...) -> None: ...

class Position(_message.Message):
    __slots__ = ["theta", "x", "y"]
    THETA_FIELD_NUMBER: _ClassVar[int]
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    theta: _containers.RepeatedScalarFieldContainer[float]
    x: _containers.RepeatedScalarFieldContainer[float]
    y: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, x: _Optional[_Iterable[float]] = ..., y: _Optional[_Iterable[float]] = ..., theta: _Optional[_Iterable[float]] = ...) -> None: ...

class Proximity(_message.Message):
    __slots__ = ["closest_distance", "status"]
    CLOSEST_DISTANCE_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    closest_distance: float
    status: ProximityStatus
    def __init__(self, closest_distance: _Optional[float] = ..., status: _Optional[_Union[ProximityStatus, str]] = ...) -> None: ...

class ProximityStatus(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
