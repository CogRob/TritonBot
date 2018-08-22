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

Quiz = collections.namedtuple(
    "Quiz",
    ["identifier", "question_texts", "lcs_answer_templates",
     "known_wrong_answer_lcs_templates", "correct_answers"])

_QUIZ = [
    Quiz("white_house",
         # "leave" = "live", TTS pronounces live as "l[ai]ve".
         ["Where does the president of the United States of America leave?"],
         ["White(20) House(20)", "At the White House", "United States",
          "Washington DC", "DC(100)", "D C", "Washington D C", "Washington"],
         ["Alabama", "Alaska", "Arizona", "Arkansas", "California", "Colorado",
          "Connecticut", "Delaware", "Florida", "Georgia", "Hawaii", "Idaho",
          "Illinois", "Indiana", "Iowa", "Kansas", "Kentucky", "Louisiana",
          "Maine", "Maryland", "Massachusetts", "Michigan", "Minnesota",
          "Mississippi", "Missouri", "Montana", "Nebraska", "Nevada",
          "New Hampshire", "New Jersey", "New Mexico", "New York",
          "North Carolina", "North Dakota", "Ohio", "Oklahoma", "Oregon",
          "Pennsylvania", "Rhode Island", "South Carolina", "South Dakota",
          "Tennessee", "Texas", "Utah", "Vermont", "Virginia", "West Virginia",
          "Wisconsin", "Wyoming", "China", "India", ],
         [("The president of the United States of America lives at the White "
           "House in Washington DC."), ]),
    Quiz("pennsylvania",
         ["What is the only US state that starts with the letter, 'P'?"],
         ["Pennsylvania", "Penn"],
         ["Alabama", "Alaska", "Arizona", "Arkansas", "California", "Colorado",
          "Connecticut", "Delaware", "Florida", "Georgia", "Hawaii", "Idaho",
          "Illinois", "Indiana", "Iowa", "Kansas", "Kentucky", "Louisiana",
          "Maine", "Maryland", "Massachusetts", "Michigan", "Minnesota",
          "Mississippi", "Missouri", "Montana", "Nebraska", "Nevada",
          "New Hampshire", "New Jersey", "New Mexico", "New York",
          "North Carolina", "North Dakota", "Ohio", "Oklahoma", "Oregon",
          "Rhode Island", "South Carolina", "South Dakota",
          "Tennessee", "Texas", "Utah", "Vermont", "Virginia", "Washington",
          "West Virginia", "Wisconsin", "Wyoming", ],
         ["Pennsylvania is the only state that starts with the letter P.", ]),
    Quiz("hot_air",
         ["Is hot air lighter, or heavier, than cold air."],
         ["lighter(100)"],
         ["heavier(100)"],
         ["Hot air is lighter than cold air.", ]),
    Quiz("soccer",
         ["What is the most popular sport throughout the world?"],
         ["soccer(100)", "football(100)"],
         ["archery", "badminton", "baseball", "softball", "basketball",
          "beach volleyball", "boxing", "canoe", "kayak", "climbing", "cycling",
          "diving", "equestrian", "fencing", "field hockey", "golf",
          "gymnastics", "handball", "judo", "karate", "modern pentathlon",
          "roller sport", "rowing", "rugby", "sailing", "shooting", "swimming",
          "surfing", "synchronized swimming", "table tennis", "taekwondo",
          "tennis", "track and field", "triathlon", "volleyball", "water polo",
          "weightlifting", "wrestling", ],
         ["Soccer is the most popular sport throughout the world.", ]),
    Quiz("keyboard_s",
         [("What letter is located between letter, A, and, D, on a computer "
          "keyboard?")],
         ["S(100)", "as", "es", "yes", "ass", "ask", "as.*", "es.*"],
         ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N",
          "O", "P", "Q", "R", "T", "U", "V", "W", "X", "Y", "Z", ],
         ["S is located between letter A and D.", ]),
]

# _QUIZ = [
#     Quiz("triton",
#          ["who is the official mascot of u c san diego?"],
#          ["Triton(10)", "King Triton(10)", "Son(5) of Poseidon(5)", ],
#          ["the official mascot of u c san diego is king triton", ]),
#     Quiz("chancellor",
#          ["who is the current chancellor of u c san diego?"],
#          ["Pradeep(10)", "Khosla(10)", "pradeep(10) khosla(10)", ],
#          ["current chancellors name is pradeep khosla", ]),
#     Quiz("geisel",
#          ["who is the geisel library named after?"],
#          ["Doctor Seuss(10)", "Suess(10)", "Theodor(10) Seuss(10)", ],
#          ["geisel library is named after doctor suess", ]),
#     Quiz("six_colleges",
#          ["do you know how many colleges is u c san diego made of?"],
#          ["Six(10)", "It is made up of six(8) colleges(8)",
#           "6(10) colleges(2)", "6", ],
#          ["u c san diego is made up of six colleges. namely, revelle, muir, "
#           "marshall, warren, roosevelt and sixth college", ]),
#     Quiz("surfing",
#          ["apart from technology, u c san diego ranks first in another field. "
#           "do you know what that is? hint: think non-academia"],
#          ["Surf(5)", "Surfing(5)", "it is a top surfing(5) school", ],
#          ["u c san diego is the top surfing school in the country", ]),
#     Quiz("sun_god",
#          ["u c san diego has an annual music festival. i know, very cool. "
#           "can you tell me its name?"],
#          ["sun(4) god(4)", "the sun(4) god(4) music festival", ],
#          ["the name is sun god music festival. it is actually named after the "
#           "sun god statue on campus.", ]),
#     Quiz("scream_five_minutes",
#          ["during finals week muir college holds something called a primal "
#           "scream. do you know what that is?"],
#          ["students(2) scream(4) together for five(2) minutes",
#           "screaming(4) students(1)", "five minutes"],
#          ["primal scream is a 5 minute break when students gather at muir "
#           "college and scream together", ]),
#     Quiz("black_beach",
#          ["U C San Diego students enjoy direct beach access from the campus. "
#           "Perchance would you know of its name?"],
#          ["black(5) beach(3)", "it is the black(5) beach(3)", ],
#          ["it is the black beach", ]),
#     Quiz("poseidon",
#          ["time for a curveball, do you know who is the father of the U C San "
#           "Diego mascot?"],
#          ["Poseidon(10)", "Greek(2) god(2) of sea(2)", ],
#          ["it is the ancient Greek God of the sea, Poseidon", ]),
#     Quiz("eucalyptus_trees",
#          ["U C San Diego has more 220,000 eucalyptus trees on campus. Do you "
#           "know their original purpose?"],
#          ["Railroad(5) and ship(5) construction",
#           "Railroads(5) and ships(5) construction",
#           "Railroad(5) and ship(5) building",
#           "Railroads(5) and ships(5) building",
#           "Railroad(3) ties(2) and ship(3) building(2)", ],
#          ["the wood was used for railroad ties and ship building!", ]),
# ]

def GetAllQuiz():
  # Makes a copy, otherwise caller may mutate the returned value and affect the
  # global variable.
  return list(_QUIZ)
