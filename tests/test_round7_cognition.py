import numpy as np

from cortex.cerebellum import CerebellumForwardModel
from affect.emotion_agent import EmotionAgent
from training.curriculum_runner import CurriculumRunner
from cortex.cortical_agent import CorticalAgent
from envs.base_env import BaseEnv
from perception.vision_agent import VisionAgent
from perception.auditory_agent import AuditoryAgent
from memory.memory_agent import MemoryAgent
import pybullet as pb


def test_forward_model_reduces_error():
    model = CerebellumForwardModel(2, 2, lr=0.1)
    state = np.zeros(2)
    act = np.ones(2)
    next_state = np.array([0.5, -0.5])
    err0 = model.update(state, act, next_state)
    for _ in range(1000):
        model.update(state, act, next_state)
    err1 = model.update(state, act, next_state)
    assert err1 < err0


def test_emotion_modulates_lr():
    emo = EmotionAgent(valence=0.5, arousal=1.0, base_lr=0.01)
    assert np.isclose(emo.modulated_lr(), 0.01 * 1.5 * 2.0)


def test_cortex_balances():
    env = BaseEnv()
    agent = CorticalAgent(env.observation_space, env.action_space)
    agent.train(env, 100)
    obs, _ = env.reset()
    done = False
    steps = 0
    while not done and steps < 100:
        action = agent.infer(obs)
        obs, _, done, _, _ = env.step(action)
        steps += 1
    assert steps >= 50


def test_curriculum_progresses():
    env = BaseEnv()
    agent = CorticalAgent(env.observation_space, env.action_space)
    runner = CurriculumRunner(agent)
    results = runner.run(steps_per_phase=10)
    assert sum(results) >= 2


def test_vision_agent_capture_shape():
    cid = pb.connect(pb.DIRECT)
    agent = VisionAgent()
    frame = agent.capture()
    pb.disconnect()
    assert frame.shape == (agent.height, agent.width)


def test_auditory_agent_process_shape():
    agent = AuditoryAgent(n_bands=16)
    waveform = np.sin(np.linspace(0, 2 * np.pi, 100))
    mag, cues = agent.process(waveform)
    assert mag.shape == (16,)
    assert cues.shape == (16,)


def test_memory_buffer_rollover():
    mem = MemoryAgent(capacity=2)
    for i in range(3):
        mem.add(np.array([i]), np.array([i]), float(i), np.array([i + 1]))
    assert len(mem.buffer) == 2
    assert mem.idx == 1
