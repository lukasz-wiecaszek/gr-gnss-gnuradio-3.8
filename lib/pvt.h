/* -*- c++ -*- */
/*
 * Copyright 2021 Lukasz Wiecaszek <lukasz.wiecaszek@gmail.com>.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_GNSS_PVT_H
#define INCLUDED_GNSS_PVT_H

#include <cstdio>
#include <string>

#include "vector3d.h"
#include "gnss_parameters.h"

namespace gr {
  namespace gnss {
    namespace pvt {

      struct satelite
      {
        vector3d position; // satelite position
        double pseudorange; // associated pseudorange measurement

        satelite()
        {
        }

        std::string to_string() const
        {
          char strbuf[1024];

          snprintf(strbuf, sizeof(strbuf),
            "%s pseudorange: %+.15e", gr::gnss::to_string(position).c_str(), pseudorange);

          return std::string(strbuf);
        }

        operator std::string() const
        {
          return to_string();
        }
      };

      struct user
      {
        vector3d position; // user/receiver position
        double dt; // advance of the user/reveiver clock with respect to system time

        user() :
          position{{0.0, 0.0, 0.0}},
          dt{0.0}
        {
        }

        std::string to_string() const
        {
          char strbuf[1024];

          snprintf(strbuf, sizeof(strbuf),
            "%s dt: %+.15e", gr::gnss::to_string(position).c_str(), dt);

          return std::string(strbuf);
        }

        operator std::string() const
        {
          return to_string();
        }
      };

      int get(const satelite* s, std::size_t N, user& hint, vector3d* position, vector3d* velocity, double* t)
      {
        lts::tensor<double, lts::dimensions<4>, lts::dimensions<4>> H;
        lts::tensor<double, lts::dimensions<4>, lts::dimensions<>> dq;
        lts::tensor<double, lts::dimensions<4>, lts::dimensions<>> dp{{0.0, 0.0, 0.0, 0.0}};

        do {
          for (int n = 0; n < 4; ++n) {
            vector3d r = s[n].position - hint.position;
            double r_magnitude = abs(r);
            double u_pseudorange = r_magnitude + C * hint.dt; // user/approximated pseudorange
            dq.set({n}, {}, u_pseudorange - s[n].pseudorange);
            for (int i = 0; i < 3; ++i)
              H.set({n}, {i}, (s[n].position.get({i}, {}) - hint.position.get({i}, {})) / r_magnitude);
            H.set({n}, {3}, 1);
          }

          double det = H.det();
          if (det != 0)
            dp = inverse(H, det) * dq;
          else
            break;

          hint.position.set({0}, {}, hint.position.get({0}, {}) + dp.get({0}, {}));
          hint.position.set({1}, {}, hint.position.get({1}, {}) + dp.get({1}, {}));
          hint.position.set({2}, {}, hint.position.get({2}, {}) + dp.get({2}, {}));
          hint.dt = hint.dt - dp.get({3}, {}) / C;
        } while (std::abs(dp.get({0}, {})) > 1.0 ||
                 std::abs(dp.get({1}, {})) > 1.0 ||
                 std::abs(dp.get({2}, {})) > 1.0 ||
                 std::abs(dp.get({3}, {})) > 1.0);

        if (position)
          *position = hint.position;

        if (velocity)
          ;// not supported yet

        if (t)
          *t = hint.dt;

        return 0;
      }

      template<std::size_t N>
      int get(const satelite (&s)[N], user& hint, vector3d* position, vector3d* velocity, double* t)
      {
        return get(s, N, hint, position, velocity, t);
      }

      template<std::size_t N>
      int get(const std::array<satelite, N>& s, user& hint, vector3d* position, vector3d* velocity, double* t)
      {
        return get(s, N, hint, position, velocity, t);
      }

    } // namespace pvt
  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_PVT_H */

