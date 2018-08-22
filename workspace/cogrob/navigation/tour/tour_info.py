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

TourPoint = collections.namedtuple(
    "TourPoint",
    ["id", "nav_point", "short_name", "intro_skip_short_name", "lcs_templates",
     "intro_words"])

_TOUR_INFO_ATK_SIX = [
  TourPoint(
      "henrik", "PSP", "Henrik's Office", False,
      ["henrik(10)", "henriks(10)", "christensen(10)", "dr robot", "CRI(10)",
       "director of contextual robotics institue", "director of CRI"],
      ("Dr. Henrik Christensen is the director of the Institute for Contextual "
       "Robotics. He is also the Qualcomm Chancellor's Chair of Robot Systems "
       "and a Professor of Computer Science at Deparment of Computer Science "
       "and Engineering UC San Diego.")),
  TourPoint(
      "cogrob_students", "SAN", "Robotic Students' Desks", False,
      ["robotic(10) students(10)", "robotics(10) students(10)", "cogrob(10)",
       "henrik(10) students(10)", "shengye(10)", "priyam(10)", "ruffin(10)",
       "antonella(10)", "ali(10)", "shixin(10)", "danbing(10)"],
      ("The robotics students sits here. Shengye Wang works on robust and "
       "reliable autonomous systems. Priyam Parashar is interested in social "
       "autonomy for service robots. Ruffin White focuses on robotics software "
       "engineering. Antonella Wilby's theis topic is underwater multi-robot "
       "SLAM.")),
  TourPoint(
      "restrooms", "GJT", "Restrooms", False,
      ["restroom", "rest room", "toilet", "W C"], "Feel free to use them."),
  TourPoint(
      "leah", "FLL", "Leah's Office", False,
      ["leah(10)"],
      "Leah Kent is the project manager of Contextual Robotics Institute"),
  TourPoint(
      "riek", "BDL", "Laurel's Office", False,
      ["Laurel(10)", "Riek(10)", "Laurel Riek", "Dr Riek"],
      ("Dr. Riek is a roboticist with interests in human-robot teaming, "
       "computer vision, and healthcare engineering, and focuses on building "
       "systems able to sense, respond, and adapt to people. Riek's current "
       "research projects have applications in critical care, "
       "neurorehabilitation, and manufacturing. Dr. Laurel Riek is an "
       "Associate Professor of Computer Science and Engineering at UC San "
       "Diego, and also has an appointment in the Department of Emergency "
       "Medicine.")),
  TourPoint(
      "hylton", "SJC", "Todd's Office ", False,
      ["Todd(10)", "Hylton(10)", "Todd Hylton", "Executive(5) Director of CRI",
       "Executive(5) Director of Contextual Robotics Institute"],
      ("Dr. Todd Hylton is a professor of practice, machine learning "
       "algorithms, and natural intelligence. He has a track record of "
       "creating successful programs and products both in government and "
       "industry, including a multi-million dollar DARPA effort to create a "
       "neuromorphic chip. His research interests include machine learning "
       "algorithms and natural intelligence.")),
  TourPoint(
      "elevator", "LAX", "The Elevator", False,
      ["Elevator(10)", "Lift(10)"],
      ("I am still trying to learn to operate the evelvator")),
]

_TOUR_INFO_ATK_ONE = [
  TourPoint(
      "prototyping_lab", "JNU", "The Prototyping Lab", False,
      ["Prototyping(10) Lab"],
      ("The prototyping lab provide prototyping and engineering services to "
       "UCSD researchers, including embedded electronics, mechanical devices, "
       "robotic contraptions, real-time networking software, and 3D printed "
       "structures.")),
  TourPoint(
      "robot_zoo", "PHX", "The Robot Zoo", True,
      ["Robot(10) Zoo(10)"],
      ("Here is the robotic zoo. You can see my fellow robot, Diego-san. He "
       "can see people, understand gestures and expressions, and even learn "
       "from people. Diego-san is not the only robot here, many of my other "
       "fellow robots are coming very soon.")),
  TourPoint(
      "smart_home", "LIT", "The Smart Home Demo Room", False,
      ["Smart(10) Home(10)"],
      ("We are shaping the future the look of a modern family. Can you imagine "
       "in the future, there will be smart sensors, voice assistants, and even "
       "robots in your house. I wish I can serve you someday.")),
  TourPoint(
      "picture_1", "PVG", "Picture First from Left", True, [],
      ("Here is an art exhibition from Trish Stone. A network error occurs "
       "when there is a software malfunction or replication of virus.")),
  TourPoint(
      "picture_2", "DHN", "Picture Second from Left", True, [],
      ("For network error, Trish Stone confronts the social, cultural, "
       "institutional network with regard to the virality of her body.")),
  TourPoint(
      "picture_3", "MGM", "Picture Third from Left", True, [],
      ("She has created a squad of 3D printed selfies and used them to stage "
       "tiny protests in public spaces.")),
  TourPoint(
      "picture_4", "FAI", "Picture Forth from Left", True, [],
      ("By replicating her body in miniature she iteratively infects and "
       "assaults the allegory of the network, calling for evolution as a "
       "cure.")),
  TourPoint(
      "elevator_entrance", "PEK", "Elevator Entrance", True, ["Elevator(10)"],
      ("Here is the elevator and it is our last stop. Thanks for letting me "
       "show you around.")),
]

_TOUR_INFO_ATK_6204 = [
  TourPoint(
      "upper_left_corner", "SAN", "The Upper Left Corner", False,
      ["The Upper Left Corner"],
      ("It is the corner near a charger, although I am not sure if the charger "
       "is still there")),
  TourPoint(
      "lower_left_corner", "DHN", "The Lower Left Corner", False,
      ["The Lower Left Corner"],
      ("It is the corner near the soldering station. The developers should "
       "jump into the maze from here")),
  TourPoint(
      "lower_right_corner", "HSV", "The Lower Right Corner", False,
      ["The Lower Right Corner"],
      ("It is the corner with another charger")),
  TourPoint(
      "upper_right_corner", "PVG", "The Upper Right Corner", False,
      ["The Upper Right Corner"],
      ("It is the corner that made of real walls")),
]


_TOUR_INFO_PER_LOCATION = {
  "atk_six": _TOUR_INFO_ATK_SIX,
  "atk_one": _TOUR_INFO_ATK_ONE,
  "atk_6204": _TOUR_INFO_ATK_6204,
}


def GetAllLocations():
  return _TOUR_INFO_PER_LOCATION.keys()


def GetAllTourPoints(location="atk_one"):
  return _TOUR_INFO_PER_LOCATION[location]


def GetAllTourPointsMap(location="atk_one"):
  return {i.id: i for i in _TOUR_INFO_PER_LOCATION[location]}
