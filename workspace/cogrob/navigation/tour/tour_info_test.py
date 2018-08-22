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

import unittest

from cogrob.navigation.tour import tour_info

class TestAllTourPoints(unittest.TestCase):
  def testGetAllTourPoints(self):
    for location in tour_info.GetAllLocations():
      all_tour_points = tour_info.GetAllTourPoints(location)
      for tour_point in all_tour_points:
        self.assertTrue(isinstance(tour_point, tour_info.TourPoint))
        self.assertTrue(isinstance(tour_point.id, str))
        self.assertTrue(bool(tour_point.id))

        self.assertTrue(isinstance(tour_point.nav_point, str))
        self.assertEqual(len(tour_point.nav_point), 3)

        self.assertTrue(isinstance(tour_point.short_name, str))
        self.assertGreater(len(tour_point.nav_point), 0)

        self.assertTrue(isinstance(tour_point.lcs_templates, list))
        for lcs_template in tour_point.lcs_templates:
          self.assertTrue(isinstance(lcs_template, str))
          self.assertTrue(bool(lcs_template))

        self.assertTrue(isinstance(tour_point.intro_words, str))
        self.assertGreater(len(tour_point.intro_words), 0)


  def testGetAllTourPointsMap(self):
    for location in tour_info.GetAllLocations():
      all_tour_points_map = tour_info.GetAllTourPointsMap(location)
      self.assertTrue(isinstance(all_tour_points_map, dict))
      for k, v in all_tour_points_map.items():
        self.assertTrue(isinstance(v, tour_info.TourPoint))
        self.assertEqual(k, v.id)


if __name__ == '__main__':
  unittest.main()
