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

import collections

from cogrob.dialogue.intent import hypothesis_pb2
from cogrob.dialogue.intent_extractors import extractor_base
from cogrob.dialogue.intent_util import lcs_matcher

COMMON_US_NAMES = [
    "James", "Mary", "John", "Patricia", "Robert", "Jennifer", "Michael",
    "Elizabeth", "William", "Linda", "David", "Barbara", "Richard", "Susan",
    "Joseph", "Jessica", "Thomas", "Margaret", "Charles", "Sarah",
    "Christopher", "Karen", "Daniel", "Nancy", "Matthew", "Betty", "Anthony",
    "Lisa", "Donald", "Dorothy", "Mark", "Sandra", "Paul", "Ashley", "Steven",
    "Kimberly", "Andrew", "Donna", "Kenneth", "Carol", "George", "Michelle",
    "Joshua", "Emily", "Kevin", "Amanda", "Brian", "Helen", "Edward",
    "Melissa", "Ronald", "Deborah", "Timothy", "Stephanie", "Jason", "Laura",
    "Jeffrey", "Rebecca", "Ryan", "Sharon", "Gary", "Cynthia", "Jacob",
    "Kathleen", "Nicholas", "Amy", "Eric", "Shirley", "Stephen", "Anna",
    "Jonathan", "Angela", "Larry", "Ruth", "Justin", "Brenda", "Scott",
    "Pamela", "Frank", "Nicole", "Brandon", "Katherine", "Raymond",
    "Virginia", "Gregory", "Catherine", "Benjamin", "Christine", "Samuel",
    "Samantha", "Patrick", "Debra", "Alexander", "Janet", "Jack", "Rachel",
    "Dennis", "Carolyn", "Jerry", "Emma", "Tyler", "Maria", "Aaron",
    "Heather", "Henry", "Diane", "Douglas", "Julie", "Jose", "Joyce", "Peter",
    "Evelyn", "Adam", "Frances", "Zachary", "Joan", "Nathan", "Christina",
    "Walter", "Kelly", "Harold", "Victoria", "Kyle", "Lauren", "Carl",
    "Martha", "Arthur", "Judith", "Gerald", "Cheryl", "Roger", "Megan",
    "Keith", "Andrea", "Jeremy", "Ann", "Terry", "Alice", "Lawrence", "Jean",
    "Sean", "Doris", "Christian", "Jacqueline", "Albert", "Kathryn", "Joe",
    "Hannah", "Ethan", "Olivia", "Austin", "Gloria", "Jesse", "Marie",
    "Willie", "Teresa", "Billy", "Sara", "Bryan", "Janice", "Bruce", "Julia",
    "Jordan", "Grace", "Ralph", "Judy", "Roy", "Theresa", "Noah", "Rose",
    "Dylan", "Beverly", "Eugene", "Denise", "Wayne", "Marilyn", "Alan",
    "Amber", "Juan", "Madison", "Louis", "Danielle", "Russell", "Brittany",
    "Gabriel", "Diana", "Randy", "Abigail", "Philip", "Jane", "Harry",
    "Natalie", "Vincent", "Lori", "Bobby", "Tiffany", "Johnny", "Alexis",
    "Logan", "Kayla",
]

FAMILIAR_NAMES = [
  "Henrik", "Ruffin", "Priyam", "Shengye",
]

NAME_HINTS = COMMON_US_NAMES + FAMILIAR_NAMES

class NameExtractor(extractor_base.ExtractorBase):

  def __init__(self):
    super(NameExtractor, self).__init__()
    self._lcs_matcher = lcs_matcher.LcsMatcher()


  def ExtractImpl(self, transcript, metadata):
    del metadata

    templates = [
        "my(10) name(10) is '\w+'<name>",
        "my(10) name(10) is '\w+'<name> '\w+'<name2>",
        "name(10) is '\w+'<name>",
        "name(10) is '\w+'<name> '\w+'<name2>",
        "Im(10) '\w+'<name> '\w+'<name2>",
        "I(10) am(10) '\w+'<name>",
        "I(10) am(10) '\w+'<name> '\w+'<name2>",
        "'\w+'<name>(.1)",
        "'\w+'<name>(.1) '\w+'<name2>(.1)",
        "'{}'<name>(100)".format("|".join(NAME_HINTS))
    ]

    hypothesis = None
    match_result = self._lcs_matcher.MatchMultiTemplateStr(
        templates, transcript)

    if (match_result is not None
        and "name" in match_result.fields
        and match_result.fields["name"].strip() != ""):
      if (hypothesis is None or
          hypothesis.metadata.confidence_score < match_result.likelihood):
        hypothesis = hypothesis_pb2.IntentHypothesis()
        name = match_result.fields["name"]
        if "name2" in match_result.fields:
          name += " " + match_result.fields["name2"]
        hypothesis.intent.human_name.name = name
        if name in COMMON_US_NAMES:
          hypothesis.intent.human_name.common_name = True
        if name in FAMILIAR_NAMES:
          hypothesis.intent.human_name.familiar_name = True
        hypothesis.metadata.confidence_score = match_result.likelihood

    if hypothesis is not None:
      return [hypothesis]

    return []
