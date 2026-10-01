"""1.1 大模型客户端：抽象接口 + DeepSeek 实现 + 本地桩实现（无 ROS 依赖，便于单测）。"""

import json
import os
import time
import urllib.request
from typing import Any, Dict, Optional, Tuple

from .goal_schema import validate_goal_schema

DEEPSEEK_BASE_URL = "https://api.deepseek.com"
DEEPSEEK_MODEL = "deepseek-chat"


def build_goal_prompt(context: Dict[str, Any]) -> Tuple[str, str]:
    """构建 system/user 提示词，要求模型只输出结构化 JSON。"""
    system = (
        "你是巡检四足机器人的任务规划器。只能输出一个 JSON 对象，字段必须为："
        "goal_id(string)、task_type(inspection|abnormal|collection)、"
        "target_pose({x,y,z:number})、constraints(string)、valid_until_sec(number)。"
        "不要输出 JSON 以外的任何内容。"
    )
    user = json.dumps(
        {
            "instruction": context.get("instruction") or {},
            "alerts": context.get("alerts") or [],
            "robot_state": context.get("robot_state") or {},
        },
        ensure_ascii=False,
    )
    return system, user


def parse_goal_response(text: Optional[str]) -> Optional[Dict[str, Any]]:
    """从模型回复中提取 JSON 并校验为结构化目标。"""
    if not text:
        return None
    text = text.strip()
    if text.startswith("```"):
        text = text.strip("`")
        if text.startswith("json"):
            text = text[4:]
    start = text.find("{")
    end = text.rfind("}")
    if start < 0 or end <= start:
        return None
    try:
        goal = json.loads(text[start:end + 1])
    except json.JSONDecodeError:
        return None
    ok, _ = validate_goal_schema(goal)
    return goal if ok else None


class LlmClient:
    """抽象大模型接口。"""

    def generate_goal(self, context: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        raise NotImplementedError


class StubLlmClient(LlmClient):
    """无后端模型时，依据指令/告警生成固定结构化目标。"""

    def __init__(self) -> None:
        self._seq = 0

    def generate_goal(self, context: Dict[str, Any]) -> Dict[str, Any]:
        self._seq += 1
        instruction = context.get("instruction") or {}
        alerts = context.get("alerts") or []
        task_type = "abnormal" if alerts else "inspection"
        raw = instruction.get("raw_content") or (
            alerts[0].get("alert_type") if alerts else "default"
        )
        return {
            "goal_id": f"goal-{self._seq}",
            "task_type": task_type,
            "target_pose": {"x": 0.0, "y": 0.0, "z": 0.0},
            "constraints": f"from instruction: {raw}",
            "valid_until_sec": time.time() + 30.0,
        }


class DeepseekLlmClient(LlmClient):
    """DeepSeek OpenAI 兼容接口实现。API Key 从参数或环境变量 DEEPSEEK_API_KEY 读取。"""

    def __init__(
        self,
        api_key: Optional[str] = None,
        base_url: str = DEEPSEEK_BASE_URL,
        model: str = DEEPSEEK_MODEL,
        timeout: float = 30.0,
    ) -> None:
        self.api_key = api_key or os.environ.get("DEEPSEEK_API_KEY", "")
        if not self.api_key:
            raise ValueError("DEEPSEEK_API_KEY is not set")
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.timeout = timeout

    def generate_goal(self, context: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        system, user = build_goal_prompt(context)
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": user},
            ],
            "response_format": {"type": "json_object"},
            "temperature": 0.2,
        }
        request = urllib.request.Request(
            f"{self.base_url}/chat/completions",
            data=json.dumps(payload).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.api_key}",
            },
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            body = json.loads(response.read().decode("utf-8"))
        content = body["choices"][0]["message"]["content"]
        return parse_goal_response(content)
