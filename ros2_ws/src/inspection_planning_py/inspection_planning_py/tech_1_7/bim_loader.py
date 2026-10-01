"""1.7 BIM 先验装载抽象接口。

PPT 范围：基于先验 BIM 与实时 SLAM 的语义地图动态标注。

红线：BIM 格式尚未获批，bim_loader 走抽象接口
（类似 AbstractLocalizerBackend / AbstractMpcSolver），不锁具体 BIM 库。

FakeBimLoader 只做确定性输出，不虚构 BIM 构件：
- 未装入任何 BIM 时返回空列表（让 C++ 侧栅格 weight=0，不虚构）
- 已装入的 BIM 元素如实返回
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class BimElement:
    """BIM 构件（与 C++ 侧 BimElement 对齐）。"""
    id: str = ""
    center_x: float = 0.0
    center_y: float = 0.0
    center_z: float = 0.0
    prior_weight: float = 0.5  # BIM 先验置信度 [0,1]


class AbstractBimLoader(ABC):
    """BIM 装载抽象接口。

    BIM 格式未批准，与 AbstractMpcSolver / AbstractLocalizerBackend 同理：
    项目方批准具体 BIM 库（IFC/Revit/自定义）后创建派生实现。
    未批准前使用 FakeBimLoader，不虚构 BIM 数据。
    """

    @abstractmethod
    def initialize(self) -> bool:
        """初始化装载器（打开 BIM 源等）。"""

    @abstractmethod
    def is_ready(self) -> bool:
        """是否就绪。"""

    @abstractmethod
    def load_all(self) -> List[BimElement]:
        """装载全部 BIM 构件。未装入时返回空列表（不虚构）。"""

    @abstractmethod
    def find_by_id(self, bim_id: str) -> Optional[BimElement]:
        """按 ID 查找 BIM 构件。不存在时返回 None。"""


class FakeBimLoader(AbstractBimLoader):
    """测试用 BIM 装载假实现。

    确定性输出，不虚构 BIM 数据：
    - 未注入任何 BIM 元素时 load_all() 返回空列表
    - 已注入的元素如实返回
    """

    def __init__(self) -> None:
        self._elements: List[BimElement] = []
        self._initialized: bool = False

    def initialize(self) -> bool:
        self._initialized = True
        return True

    def is_ready(self) -> bool:
        return self._initialized

    def set_elements(self, elements: List[BimElement]) -> None:
        """注入测试用 BIM 元素（不虚构，仅返回已注入数据）。"""
        self._elements = list(elements)

    def load_all(self) -> List[BimElement]:
        if not self._initialized:
            return []
        return list(self._elements)

    def find_by_id(self, bim_id: str) -> Optional[BimElement]:
        for e in self._elements:
            if e.id == bim_id:
                return e
        return None
