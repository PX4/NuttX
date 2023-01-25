/****************************************************************************
 * arch/arm/src/imxrt/imxrt117x_daisy.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/*  Based on chip selection this file is included in imxrt_daisy.c */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DAISY_INDEX_INVALID     255
#define DAISY_SEL_INVALID       255
#define ALT0                    0
#define ALT1                    1
#define ALT2                    2
#define ALT3                    3
#define ALT4                    4
#define ALT5                    5
#define ALT6                    6
#define ALT7                    7
#define ALT8                    8
#define ALT9                    9
#define ALT10                   10
#define ALT11                   11

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct imxrt_daisy_entry_t
{
  uint8_t   index;
  uint8_t   sel;
};

struct imxrt_daisy_t
{
  struct imxrt_daisy_entry_t alts[12];
};

static const struct imxrt_daisy_t g_daisy_select[] =
{
  /* index:0 GPIO_EMC_B1_00 */

  {
    {
      /* Index:0 Alt:0   No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:0 Alt:1   No input selection */

      [ALT1].index  = DAISY_INDEX_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:2   No input selection */

      [ALT2].index  = DAISY_INDEX_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:3   No input selection */

      [ALT3].index  = DAISY_INDEX_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:4   No input selection */

      [ALT4].index  = DAISY_INDEX_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:5   No input selection */

      [ALT5].index  = DAISY_INDEX_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:6   No input selection */

      [ALT6].index  = DAISY_INDEX_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:7   No input selection */

      [ALT7].index  = DAISY_INDEX_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:8   No input selection */

      [ALT8].index  = DAISY_INDEX_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:9   No input selection */

      [ALT9].index  = DAISY_INDEX_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:10  No input selection */

      [ALT10].index = DAISY_INDEX_INVALID,
      [ALT10].sel   = DAISY_SEL_INVALID,

      /* Index:0 Alt:11  No input selection */

      [ALT11].index = DAISY_INDEX_INVALID,
      [ALT11].sel   = DAISY_SEL_INVALID,
    },
  },

  /* TO DO: Add for 174 pins... :( */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
