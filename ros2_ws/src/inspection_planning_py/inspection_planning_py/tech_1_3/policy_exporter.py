"""策略导出器：将训练产物打包为 C++ 运行时可加载的格式

纯离线模块，不调用实机接口，不虚构权重内容。

导出产物约定（与 C++ tech_1_3/ppo_policy_loader.hpp 对齐）：
- policy_metadata.json：框架无关的版本/观测维/动作维/校验字段
- policy.<ext>：模型权重（实际内容由训练框架决定；未批准时不生成此文件）

注意：权重文件（*.pt / *.pth / *.onnx / *.bin）已在根目录 .gitignore
排除，提交代码时只允许提交 metadata 与导出脚本，不允许提交权重。
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

from .ppo_training_pipeline import TrainingConfig


METADATA_FILENAME = "policy_metadata.json"


@dataclass
class PolicyMetadataForExport:
    """与 C++ `inspection_execution::tech_1_1::PolicyMetadata` 对齐。

    C++ 字段：version / observation_dim / action_dim。
    这里额外补充校验与来源字段，便于审计；C++ 侧只读取它认识的字段。
    """
    version: str
    observation_dim: int
    action_dim: int
    policy_filename: str          # 不含目录，仅文件名，例：policy.onnx
    training_config_hash: str     # TrainingConfig 的 sha256（前16位），便于复现训练
    export_timestamp_utc: str     # ISO8601
    terrain_ids: List[str]        # 训练所用地形 ID 列表
    sim_pass_rate: float          # 导出时的仿真通行率（PPT 1.3 指标）
    extra: Dict[str, Any] = None  # 后续扩展；C++ 加载器忽略

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        if d["extra"] is None:
            d["extra"] = {}
        return d


class PolicyExporter:
    """负责把训练产物写出到 export_dir；不生成虚构权重。"""

    def __init__(self, export_dir: str) -> None:
        self.export_dir = Path(export_dir)
        self.export_dir.mkdir(parents=True, exist_ok=True)

    # ---------- 公开 API ----------

    def write_metadata(self, metadata: PolicyMetadataForExport) -> Path:
        path = self.export_dir / METADATA_FILENAME
        path.write_text(
            json.dumps(metadata.to_dict(), indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        return path

    def read_metadata(self) -> Optional[PolicyMetadataForExport]:
        path = self.export_dir / METADATA_FILENAME
        if not path.exists():
            return None
        raw = json.loads(path.read_text(encoding="utf-8"))
        return PolicyMetadataForExport(
            version=raw["version"],
            observation_dim=int(raw["observation_dim"]),
            action_dim=int(raw["action_dim"]),
            policy_filename=raw["policy_filename"],
            training_config_hash=raw["training_config_hash"],
            export_timestamp_utc=raw["export_timestamp_utc"],
            terrain_ids=list(raw.get("terrain_ids", [])),
            sim_pass_rate=float(raw.get("sim_pass_rate", 0.0)),
            extra=raw.get("extra"),
        )

    @staticmethod
    def hash_training_config(config: TrainingConfig) -> str:
        raw = config.to_json().encode("utf-8")
        return hashlib.sha256(raw).hexdigest()[:16]

    # ---------- 权重钩子 ----------

    def can_export_weights(self) -> bool:
        """导出器不虚构权重。未提供训练后端权重文件前一律返回 False。"""
        return False

    def export_weights(self, backend_checkpoint_dir: str) -> Optional[Path]:
        """把训练后端的 checkpoint 目录转为运行时权重。

         不做实现，避免引入未批准的训练框架。批准后子类覆盖。
        """
        raise NotImplementedError(
            "policy_exporter.export_weights: 训练框架未批准，暂不导出权重。"
            "仅导出 policy_metadata.json。"
        )
