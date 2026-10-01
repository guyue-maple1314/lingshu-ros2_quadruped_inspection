"""1.9 声纹模型加载：抽象接口 + 本地桩实现（不锁具体库、不虚构推理结果）。"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class AcousticModelInfo:
    version: str
    categories: List[str] = field(default_factory=list)


class AbstractAcousticModel(ABC):
    @abstractmethod
    def load(self, model_path: str) -> bool:
        ...

    @abstractmethod
    def is_loaded(self) -> bool:
        ...

    @abstractmethod
    def info(self) -> AcousticModelInfo:
        ...

    @abstractmethod
    def infer(self, audio: List[float]) -> Tuple[bool, str, float]:
        """返回 (ok, category, confidence)；无有效结果时返回 (False, "", 0.0)。"""


class FakeAcousticModel(AbstractAcousticModel):
    """确定性桩模型：不虚构故障类别。"""

    def __init__(self) -> None:
        self._loaded = False
        self._version = "fake-0"

    def load(self, model_path: str) -> bool:
        self._loaded = bool(model_path)
        return self._loaded

    def is_loaded(self) -> bool:
        return self._loaded

    def info(self) -> AcousticModelInfo:
        return AcousticModelInfo(version=self._version, categories=[])

    def infer(self, audio: List[float]) -> Tuple[bool, str, float]:
        # 桩模型不做真实推理，不输出故障类别
        return (False, "", 0.0)
