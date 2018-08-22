# Copyright (c) 2018, The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the University of California nor the
#   names of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OF THE UNIVERSITY OF CALIFORNIA
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from greeting_bot_state_machine import general_states
import greeting_bot_state_machine.robot_speak
import random
import rospy
import smach
import smach_ros

import use_cogrob_workspace
import cogrob.dialogue.quiz.all_quiz
from cogrob_listen_intent import common_intent_listener

SayText = greeting_bot_state_machine.robot_speak.SayText

class LoadQuizState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"],
                         output_keys=["available_quiz"])


  def execute(self, userdata):
    random.seed()
    userdata.available_quiz = cogrob.dialogue.quiz.all_quiz.GetAllQuiz()
    return "next"


class SelectQuizState(smach.State):
  def __init__(self):
    # SelectQuizState selects a quiz from available_quiz, removes it from
    # available_quiz, and put it into selected_quiz. If there is no quiz,
    # returns no_quiz.
    smach.State.__init__(
      self, outcomes=["next", "no_quiz"], input_keys=["available_quiz"],
      output_keys=["available_quiz", "selected_quiz"])


  def execute(self, userdata):
    if len(userdata.available_quiz) == 0:
      return "no_quiz"
    else:
      select_index = random.choice(range(len(userdata.available_quiz)))
      userdata.selected_quiz = userdata.available_quiz[select_index]
      userdata.available_quiz.pop(select_index)
      return "next"


class AskQuizState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"], input_keys=["selected_quiz"])


  def execute(self, userdata):
    assert userdata.selected_quiz is not None
    SayText(random.sample(userdata.selected_quiz.question_texts, 1)[0])
    return "next"


class CheckAnswerState(smach.State):
  def __init__(self):
    smach.State.__init__(
      self, outcomes=["correct", "wrong", "repeat", "dont_know", "silence"],
      input_keys=["selected_quiz"])


  def intent_filter_func(self, selected_quiz, intent):
    del self
    if intent.intent.HasField("quiz_answer"):
      if (intent.intent.quiz_answer.matched_answer != selected_quiz.identifier
          and intent.intent.quiz_answer.matched_wrong_answer !=
          selected_quiz.identifier):
        return None
    return intent


  def execute(self, userdata):
    listener = common_intent_listener.GetCommonIntentListener(
      ["quiz_answer", "dont_know", "say_again"],
      map_func=lambda x: self.intent_filter_func(userdata.selected_quiz, x))
    result = listener.ListenForIntent()
    if (result.intent.quiz_answer.matched_answer
          == userdata.selected_quiz.identifier):
      return "correct"
    elif (result.intent.quiz_answer.matched_wrong_answer
          == userdata.selected_quiz.identifier):
      return "wrong"
    elif result.intent.WhichOneof("intent") is None:
      return "wrong"
    elif result.intent.HasField("silence"):
      return "silence"
    elif result.intent.HasField("dont_know"):
      return "dont_know"
    elif result.intent.HasField("say_again"):
      return "repeat"
    else:
      raise NotImplementedError("Unknown intent: {}".format(str(result)))


class SayRepeatQuestionState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"])


  def execute(self, userdata):
    SayText("Let me repeat the question.")
    return "next"


class CongratulateCorrectAnswerState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"], input_keys=["selected_quiz"])


  def execute(self, userdata):
    assert userdata.selected_quiz is not None
    # TODO(shengye): Give variations for this.
    SayText("You got it!")
    SayText(random.sample(userdata.selected_quiz.correct_answers, 1)[0])
    return "next"


class RespondWrongAnswerState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"], input_keys=["selected_quiz"])


  def execute(self, userdata):
    assert userdata.selected_quiz is not None
    # TODO(shengye): Give variations for this.
    SayText("Nice try, but that was not correct.")
    SayText(random.sample(userdata.selected_quiz.correct_answers, 1)[0])
    return "next"


class RespondDontKnowState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"], input_keys=["selected_quiz"])


  def execute(self, userdata):
    assert userdata.selected_quiz is not None
    # TODO(shengye): Give variations for this.
    SayText("No worries.")
    SayText(random.sample(userdata.selected_quiz.correct_answers, 1)[0])
    return "next"


class DecideToAskWantMoreState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["ask_want_more", "no_more"],
                         input_keys=["available_quiz"])


  def execute(self, userdata):
    if len(userdata.available_quiz) == 0:
      return "no_more"
    else:
      return "ask_want_more"


class SayNoMoreQuizState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"])


  def execute(self, userdata):
    SayText("Okay, that is all the questions I have.")
    return "next"


class AskWantMoreState(smach.State):
  def __init__(self):
    smach.State.__init__(
      self, outcomes=["want_more", "no_more", "silence"])


  def execute(self, userdata):
    SayText("Do you want to try another question?")
    listener = common_intent_listener.GetCommonIntentListener(["yes_no"])
    result = listener.ListenForIntent()
    if result.intent.HasField("silence"):
      return "silence"
    elif result.intent.yes_no.is_yes:
      return "want_more"
    else:
      return "no_more"


class SayGoodbyeState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"])


  def execute(self, userdata):
    # TODO(shengye): Add more variations.
    SayText("Thanks for chatting with me. I feel honored.")
    return "next"


def GetQuizStateMachine():
  sm_quiz = smach.StateMachine(outcomes=["finished", "silence"])

  sm_quiz.userdata.available_quiz = []
  sm_quiz.userdata.selected_quiz = None
  sm_quiz.userdata.silence_retry = False

  with sm_quiz:
    smach.StateMachine.add(
        "load_quiz", LoadQuizState(),
        transitions={"next": "select_quiz"},
        remapping={"available_quiz": "available_quiz"})
    sm_quiz.set_initial_state(["load_quiz"])

    smach.StateMachine.add(
        "select_quiz", SelectQuizState(),
        transitions={"next": "ask_quiz", "no_quiz": "load_quiz"},
        remapping={"available_quiz": "available_quiz",
                   "selected_quiz": "selected_quiz"})

    smach.StateMachine.add(
        "reset_silence_retried",
        general_states.ResetRetryCounter("silence_retry", 2 - 1),
        transitions={"next": "ask_quiz"},
        remapping={"silence_retry": "silence_retry"})

    smach.StateMachine.add(
        "ask_quiz", AskQuizState(),
        transitions={"next": "check_answer"},
        remapping={"selected_quiz": "selected_quiz"})

    smach.StateMachine.add(
        "check_answer", CheckAnswerState(),
        transitions={"correct": "congratulate_correct_answer",
                     "wrong": "respond_wrong_answer",
                     "repeat": "ask_quiz",
                     "silence": "test_silence_retry",
                     "dont_know": "respond_dont_know"},
        remapping={"selected_quiz": "selected_quiz"})

    smach.StateMachine.add(
        "congratulate_correct_answer", CongratulateCorrectAnswerState(),
        transitions={"next": "decide_to_ask_want_more"},
        remapping={"selected_quiz": "selected_quiz"})

    smach.StateMachine.add(
        "respond_wrong_answer", RespondWrongAnswerState(),
        transitions={"next": "decide_to_ask_want_more"},
        remapping={"selected_quiz": "selected_quiz"})

    smach.StateMachine.add(
        "respond_dont_know", RespondDontKnowState(),
        transitions={"next": "decide_to_ask_want_more"},
        remapping={"selected_quiz": "selected_quiz"})

    smach.StateMachine.add(
        "test_silence_retry",
        general_states.DecreaseAndTestRetry("silence_retry"),
        transitions={"continue": "say_repeat_question",
                     "give_up": "silence"},
        remapping={"silence_retry": "silence_retry"})

    smach.StateMachine.add(
        "say_repeat_question", SayRepeatQuestionState(),
        transitions={"next": "ask_quiz"})

    smach.StateMachine.add(
        "decide_to_ask_want_more", DecideToAskWantMoreState(),
        transitions={"ask_want_more": "ask_want_more",
                     "no_more": "no_more_quiz"},
        remapping={"available_quiz": "available_quiz"})

    smach.StateMachine.add(
        "ask_want_more", AskWantMoreState(),
        transitions={"want_more": "select_quiz",
                     "silence": "say_goodbye",
                     "no_more": "say_goodbye"},
        remapping={"available_quiz": "available_quiz"})

    smach.StateMachine.add(
        "no_more_quiz", SayNoMoreQuizState(),
        transitions={"next": "say_goodbye"},
        remapping={"available_quiz": "available_quiz"})

    smach.StateMachine.add(
        "say_goodbye", SayGoodbyeState(),
        transitions={"next": "finished"})

  return sm_quiz
