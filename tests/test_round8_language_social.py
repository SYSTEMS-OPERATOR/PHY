import asyncio

from language.language_agent import LanguageAgent
from language.dialogue_manager import DialogueManager
from social.social_cognition import SocialCognitionAgent
from tasks.cooperative_tasks import PassTheCubeEnv
from tasks.competitive_tasks import TugOfWarEnv
from safety.alignment_layer import AlignmentSafetyLayer
from interpretability.dashboard import app
from fastapi.testclient import TestClient
from training.social_curriculum import CurriculumSocialRunner
from cortex.cortical_agent import CorticalAgent
from envs.base_env import BaseEnv


def test_language_intent():
    agent = LanguageAgent()
    intent = agent.listen("please pick up cube")
    assert intent == {"action": "pick", "obj": "cube"}


def test_dialogue_reply():
    dm = DialogueManager(emotion_agent=None)  # type: ignore
    assert dm.get_response("hello") == "Hello there!"


def test_social_predict():
    soc = SocialCognitionAgent()
    soc.update("b", [1.0, 0.0])
    assert soc.predict_actions("b", 2) == [[], []]


def test_coop_pass():
    env = PassTheCubeEnv()
    env.reset()
    _, reward, done, _, _ = env.step({"agent_0": [0.5], "agent_1": [0.5]})
    assert done and reward > 0


def test_competitive_balance():
    env = TugOfWarEnv()
    env.reset()
    _, _, done, _, _ = env.step({"agent_0": [1.0], "agent_1": [1.0]})
    assert done


def test_safety_stop():
    layer = AlignmentSafetyLayer()
    layer.trigger_stop()
    assert layer.stop


def test_dashboard_rest():
    client = TestClient(app)
    resp = client.get("/status")
    assert resp.status_code == 200


def test_social_curriculum():
    base = BaseEnv()
    agent = CorticalAgent(base.observation_space, base.action_space)
    runner = CurriculumSocialRunner(agent)
    results = runner.run(steps_per_phase=1)
    assert len(results) == 2
