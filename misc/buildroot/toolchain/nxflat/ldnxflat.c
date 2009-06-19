/***********************************************************************
 * toolchain/nxflat/ldnxflat.c
 * Convert ELF (or any BFD format) to NXFLAT binary format
 *
 * ldnxflat takes a fully resolvable elf binary which was linked with -r 
 * and resolves all references, then generates relocation table entries for
 * any relocation entries in data sections. This is designed to work with
 * the options -fpic -msingle-pic-base -mno-got (or -membedded-pic)
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 *
 * Modified from ldelf2xflat (see http://xflat.org):
 *
 *   Copyright (c) 2002, 2006, Cadenux, LLC.  All rights reserved.
 *   Copyright (c) 2002, 2006, Gregory Nutt.  All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Extended from the FLAT ldnxflat.c (original copyright below )
 *
 *   Copyright (C) 2000 NETsilicon, Inc.
 *   Copyright (C) 2000 WireSpeed Communications Corp
 *
 *   author : Joe deBlaquiere ( joe@wirespeed.com )
 *
 *   converted from elf2flt.c ( original copyright below )
 *
 *   elf2flt copyright :
 *
 *   (c) 1999, Greg Ungerer <gerg@moreton.com.au>
 *   (c) 1999, Phil Blundell, Nexus Electronics Ltd <pb@nexus.co.uk>
 *
 *   Hacked this about badly to fully support relocating binaries.
 *
 *   Originally obj-res.c
 *
 *   (c) 1998, Kenneth Albanowski <kjahds@kjahds.com>
 *   (c) 1998, D. Jeff Dionne
 *   (c) 1998, The Silver Hammer Group Ltd.
 *   (c) 1996, 1997 Dionne & Associates
 *   jeff@ryeham.ee.ryerson.ca
 *
 *   Relocation added March 1997, Kresten Krab Thorup 
 *   krab@california.daimi.aau.dk
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 ***********************************************************************/

/***********************************************************************
 * Included Files
 ***********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdarg.h>

#include <sys/types.h>
#include <netinet/in.h>

#include "bfd.h"
#include "arch/arch.h"
#include "nxflat.h"

/***********************************************************************
 * Compilation Switches
 ***********************************************************************/

/* #define RELOCS_IN_NETWORK_ORDER 1 */

#define LIBS_CAN_INCLUDE_LIBS 1

/***********************************************************************
 * Definitions
 ***********************************************************************/

#ifndef PARAMS
#  define PARAMS(x)          x
#endif

#ifdef __CYGWIN32__
#  define O_PLATFORM         O_BINARY
#else
#  define O_PLATFORM         0
#endif

#define MAX_SECTIONS       16
#define DEFAULT_STACK_SIZE 4096

#define IS_GLOBAL(x)       ((((x)->flags)&(BSF_GLOBAL))!=0)

#define NXFLAT_HDR_SIZE     sizeof(struct nxflat_hdr_s)

/* The names of these fields have changed in later versions of binutils
 * (after 2.13 and before 2.15) and the meaning of _rawsize is has also
 * changed somewhat.  In the same timeframe, the name of the section
 * structure changed.
 */

#if 0
#  define COOKED_SIZE _cooked_size
#  define RAW_SIZE    _raw_size
#  define bfd_section sec
#else
#  define COOKED_SIZE size
#  define RAW_SIZE    rawsize
#endif

/***********************************************************************
 * Private Types
 ***********************************************************************/

/* Needs to match definition in include/elf/internal.h.  This is from binutils-2.19.1 */

struct elf_internal_sym
  {
    bfd_vma       st_value;            /* Value of the symbol */
    bfd_vma       st_size;             /* Associated symbol size */
    unsigned long st_name;             /* Symbol name, index in string tbl */
    unsigned char st_info;             /* Type and binding attributes */
    unsigned char st_other;            /* Visibilty, and target specific */
    unsigned int  st_shndx;            /* Associated section index */
  };

typedef struct
  {
    /* The BFD symbol. */

    asymbol symbol;

    /* ELF symbol information.  */

    struct elf_internal_sym internal_elf_sym;

    /* Backend specific information.  */

    union
      {
        unsigned int hppa_arg_reloc;
        void *mips_extr;
        void *any;
      }
    tc_data;

    /* Version information.  This is from an Elf_Internal_Versym structure in a 
     * SHT_GNU_versym section.  It is zero if there is no version information. */

    u_int16_t version;

  } elf_symbol_type;

typedef struct _segment_info
  {
    const char *name;
    bfd_vma low_mark;
    bfd_vma high_mark;
    size_t size;
    void *contents;
    asection *subsect[MAX_SECTIONS];
    int nsubsects;
  } segment_info;

typedef void (*func_type) (asymbol * sym, void *arg1, void *arg2, void *arg3);

/***********************************************************************
 * Private Variable Data
 ***********************************************************************/

static int verbose = 0;
static int dsyms = 0;
static int stack_size = 0;

static int32_t counter = 0;

static const char *program_name = NULL;
static const char *bfd_filename = NULL;
static const char *entry_name = NULL;
static char *out_filename = NULL;

static segment_info text_info;
static segment_info data_info;
static segment_info bss_info;

static asymbol **symbol_table = NULL;
static int32_t number_of_symbols = 0;

static asymbol *entry_symbol = NULL;
static asymbol *dynname_begin_symbol = NULL;
static asymbol *dynname_end_symbol = NULL;
static asymbol *dynimport_begin_symbol = NULL;
static asymbol *dynimport_end_symbol = NULL;

/***********************************************************************
 * Private Constant Data
 ***********************************************************************/

static const char default_exe_entry_name[] = "_start";
static const char dynpath_begin_name[] = "__dynpath_begin";
static const char dynpath_end_name[] = "__dynpath_end";
static const char dynname_begin_name[] = "__dynname_begin";
static const char dynname_end_name[] = "__dynname_end";
static const char dynexport_begin_name[] = "__dynexport_begin";
static const char dynexport_end_name[] = "__dynexport_end";
static const char dynimport_begin_name[] = "__dynimport_begin";
static const char dynimport_end_name[] = "__dynimport_end";
static const char dynloader_name[] = "__dynloader";

/***********************************************************************
 * Private Functions
 ***********************************************************************/

/***********************************************************************
 * nxflat_swap32
 ***********************************************************************/

#ifdef ARCH_BIG_ENDIAN
static inline u_int32_t nxflat_swap32(u_int32_t little)
{
  u_int32_t big =
    ((little >> 24) & 0xff) |
    (((little >> 16) & 0xff) << 8) |
    (((little >> 8) & 0xff) << 16) | ((little & 0xff) << 24);
  return big;
}
#endif

/***********************************************************************
 * get_xflat32
 ***********************************************************************/

static inline u_int32_t get_xflat32(u_int32_t * addr32)
{
  return ntohl(*addr32);
}

/***********************************************************************
 * put_xflat32
 ***********************************************************************/

static void inline put_xflat32(u_int32_t * addr32, u_int32_t val32)
{
  *addr32 = htonl(val32);
}

/***********************************************************************
 * put_xflat16
 ***********************************************************************/

static void inline put_xflat16(u_int16_t * addr16, u_int16_t val16)
{
#if 1
  *addr16 = htons(val16);
#else
  u_int32_t *addr32 = (u_int32_t *) (((u_int32_t) addr16) & ~3);
  u_int32_t ndx = ((((u_int32_t) addr16) >> 1) & 1);

  union
    {
      u_int16_t hw[2];
      u_int32_t w;
    } uword;

  /* Fetch the 32 bit value */

  uword.w = get_xflat32(addr32);

  /* Add in the 16 bit value */

  uword.hw[ndx] = val16;

  /* Then save the 32-bit value */

  put_xflat32(addr32, uword.w);
#endif
}

/***********************************************************************
 * get_symbols
 ***********************************************************************/

static asymbol **get_symbols(bfd * abfd, int32_t * num)
{
  int32_t storage_needed;

  if (dsyms)
    {
      storage_needed = bfd_get_dynamic_symtab_upper_bound(abfd);
    }
  else
    {
      storage_needed = bfd_get_symtab_upper_bound(abfd);
    }

  if (storage_needed < 0)
    {
      abort();
    }

  if (storage_needed == 0)
    {
      return NULL;
    }

  symbol_table = (asymbol **) malloc(storage_needed);

  if (dsyms)
    {
      number_of_symbols = bfd_canonicalize_dynamic_symtab(abfd, symbol_table);
    }
  else
    {
      number_of_symbols = bfd_canonicalize_symtab(abfd, symbol_table);
    }

  if (number_of_symbols < 0)
    {
      abort();
    }

  *num = number_of_symbols;

  if (verbose)
    {
      printf("Read %d symbols\n", number_of_symbols);
    }

  return symbol_table;
}

/***********************************************************************
 * traverse_global_symbols
 ***********************************************************************/

static void
traverse_global_symbols(void *arg1, void *arg2, void *arg3, func_type fn)
{
  int i;
  for (i = 0; i < number_of_symbols; i++)
    {
      /* Check if it is a global function symbol defined in this */

      if (IS_GLOBAL(symbol_table[i]))
        {
          /* Yes, process the symbol */

          fn(symbol_table[i], arg1, arg2, arg3);
        }
    }
}

/***********************************************************************
 * check_special_symbol
 ***********************************************************************/

static void check_special_symbol(asymbol * sym,
                                 void *arg1, void *arg2, void *arg3)
{
  if ((entry_name) && (strcmp(entry_name, sym->name) == 0))
    {
      entry_symbol = sym;
    }
  else if (strcmp(dynname_begin_name, sym->name) == 0)
    {
      dynname_begin_symbol = sym;
    }
  else if (strcmp(dynname_end_name, sym->name) == 0)
    {
      dynname_end_symbol = sym;
    }
  else if (strcmp(dynimport_begin_name, sym->name) == 0)
    {
      dynimport_begin_symbol = sym;
    }
  else if (strcmp(dynimport_end_name, sym->name) == 0)
    {
      dynimport_end_symbol = sym;
    }
  counter++;
}

/***********************************************************************
 * find_special_symbols
 ***********************************************************************/

static void inline find_special_symbols(void)
{
  counter = 0;
  traverse_global_symbols(NULL, NULL, NULL, check_special_symbol);

  if (entry_symbol == NULL)
    {
      fprintf(stderr, "ERROR: Executable entry point \"%s\" not found\n",
              entry_name);
      exit(1);
    }

  if (dynname_begin_symbol == NULL)
    {
      fprintf(stderr, "WARNING: Special symbol \"%s\" not found\n",
              dynname_begin_name);
    }
  else
    {
      if (dynname_end_symbol == NULL)
        {
          fprintf(stderr, "ERROR: Symbol \"%s\" found, but missing \"%s\"\n",
                  dynname_begin_name, dynname_end_name);
          exit(1);
        }

#ifndef LIBS_CAN_INCLUDE_LIBS
      if (binary_type == NXFLAT_BINARY_TYPE_DYN)
        {
          fprintf(stderr,
                  "WARNING: Shared library binary contains "
                  "library names!\n");
          fprintf(stderr,
                  "         Library names cannot be used by a "
                  "shared library object\n");
          fprintf(stderr,
                  "         You may want to rebuild rebuild the "
                  "shared library intermediate file without\n");
          fprintf(strderr,
                  "         specifying Library names (-l option "
                  "in ldelflib)\n");

          dynname_begin_symbol = NULL;
          dynname_end_symbol = NULL;
        }
#endif                                 /* LIBS_CAN_INCLUDE_LIBS */
    }

  if (dynimport_begin_symbol == NULL)
    {
      fprintf(stderr, "WARNING: Special symbol \"%s\" not found\n",
              dynimport_begin_name);
      dynimport_end_symbol = NULL;
    }
  else if (dynimport_end_symbol == NULL)
    {
      fprintf(stderr, "ERROR: Symbol \"%s\" found, but missing \"%s\"\n",
              dynimport_begin_name, dynimport_end_name);
      exit(1);
    }
}

/***********************************************************************
 * put_special_symbol
 ***********************************************************************/

static void
put_special_symbol(asymbol * begin_sym, asymbol * end_sym,
                   u_int32_t * addr, u_int16_t * count,
                   u_int32_t elem_size, u_int32_t offset)
{
  u_int32_t file_offset = 0;
  u_int32_t elems = 0;

  u_int32_t begin_sym_value;
  u_int32_t begin_sect_vma;

  /* We'll assume its okay if this symbol was not found. */

  if (begin_sym != NULL)
    {
      /* Get the value of the beginning symbol and the section that it is
       * defined in. */

      begin_sym_value = begin_sym->value;
      if (begin_sym->section == NULL)
        {
          fprintf(stderr, "No section for symbol \"%s\"\n", begin_sym->name);
          exit(1);
        }
      else
        {
          /* Get the file offset to the beginning symbol */

          begin_sect_vma = begin_sym->section->vma;

          file_offset = NXFLAT_HDR_SIZE +       /* Size of the xFLT header */
            begin_sect_vma +    /* Virtual address of section */
            begin_sym_value +   /* Value of the symbol */
            offset;             /* Additional file offset */

          /* If there is a begin symbol, then there MUST be a corresponding
           * ending symbol.  We must have this to get the size of the data
           * structure.  This size will be used to determined the number of
           * elements in the array. */

          if (end_sym == NULL)
            {
              /* No matching end symbol */

              fprintf(stderr,
                      "ERROR: Begin sym \"%s\" found, no corresponding end\n",
                      begin_sym->name);
              exit(1);
            }
          else if (end_sym->section == NULL)
            {
              /* No section associated with the end symbol */

              fprintf(stderr, "No section for symbol \"%s\"\n", end_sym->name);
              exit(1);
            }
          else if (end_sym->section != begin_sym->section)
            {
              /* Section associated with the end symbol is not the same as the
               * section associated with the begin symbol. */

              fprintf(stderr,
                      "ERROR: Begin sym \"%s\" is defined in section \"%s\"\n",
                      begin_sym->name, begin_sym->section->name);
              fprintf(stderr,
                      "       but sym \"%s\" is defined in section \"%s\"\n",
                      end_sym->name, end_sym->section->name);
              exit(1);
            }
          else if (end_sym->value < begin_sym_value)
            {
              /* End symbol is before the begin symbol? */

              fprintf(stderr,
                      "ERROR: Begin sym \"%s\" lies at offset %d "
                      "in section \"%s\"\n",
                      begin_sym->name, begin_sym_value,
                      begin_sym->section->name);
              fprintf(stderr,
                      "       but sym \"%s\" is before that at offset: %d\n",
                      end_sym->name, (u_int32_t) end_sym->value);
              exit(1);
            }
          else
            {
              /* Get the size of the structure in bytes */

              u_int32_t array_size = end_sym->value - begin_sym_value;

              /* Get the number of elements in the structure. */

              elems = array_size / elem_size;

              /* Verify that there are an even number of elements in the array. */

              if (elems * elem_size != array_size)
                {
                  fprintf(stderr,
                          "ERROR: Array size (%d) is not a multiple "
                          "of the element size (%d)\n", array_size, elem_size);
                  exit(1);
                }
            }
        }

      if (verbose)
        {
          printf("Symbol %s: value: %08x section offset: %08x "
                 "file offset: %08x count: %d\n",
                 begin_sym->name, begin_sym_value, begin_sect_vma,
                 file_offset, elems);
        }
    }

  put_xflat32(addr, file_offset);
  put_xflat16(count, elems);
}

/***********************************************************************
 * put_entry_point
 ***********************************************************************/

static void put_entry_point(struct nxflat_hdr_s *hdr)
{
  u_int32_t entry_point = 0;

  if (entry_symbol)
    {
      struct bfd_section *sect;

      /* Does this symbol lie in the text section? */

      sect = entry_symbol->section;
      if (sect == NULL)
        {
          fprintf(stderr, "No section for entry symbol \"%s\"\n",
                  entry_symbol->name);
          exit(1);
        }

      /* Get the file offset to the entry point symbol */

      entry_point = NXFLAT_HDR_SIZE + sect->vma + entry_symbol->value;

      printf("Entry symbol \"%s\": %08x in section \"%s\"\n",
             entry_symbol->name, entry_point, sect->name);

      /* If verbose is selected, we'll show the details of the calculation. */

      if (verbose)
        {
          printf("(HDR: %08lx + Section VMA: %08x + Symbol Value: %08x)\n",
                 NXFLAT_HDR_SIZE, (u_int32_t) sect->vma,
                 (u_int32_t) entry_symbol->value);
        }
    }

  /* Does the entry point lie within the text region? */

  if ((entry_point < NXFLAT_HDR_SIZE) ||
      (entry_point >= NXFLAT_HDR_SIZE + text_info.size))
    {

      /* No... One special case: A shared library may not need an
       * initialization entry point. */

      if (entry_point == 0)
        {
          /* Complain a little in this case... The used might have intended to
           * specify one. */

          fprintf(stderr,
                  "WARNING: Library has no initialization entry point\n");
        }
      else
        {
          /* Otherwise, complain a lot.  We either have a program with no
           * entry_point or a bogus entry_point. */

          fprintf(stderr, "ERROR: Invalid entry point: %08x\n", entry_point);
          fprintf(stderr, "       Valid TEXT range: %08lx - %08lx\n",
                  NXFLAT_HDR_SIZE, NXFLAT_HDR_SIZE + text_info.size);
          exit(1);
        }
    }

  /* Put the entry point into the xFLT header. */

  put_xflat32(&hdr->h_entry, entry_point);
}

/***********************************************************************
 * get_reloc_type
 ***********************************************************************/

static int get_reloc_type(asection * sym_section, segment_info ** sym_segment)
{
  int i;

  /* Locate the address referred to by section type.  In the context in which
   * this runs, we can no longer use the flags field (it is zero for some
   * reason).  But we can search for matches with the buffered section
   * pointers. */

  /* Check if the symbol is defined in a BSS section */

  for (i = 0; i < bss_info.nsubsects; i++)
    {
      if (bss_info.subsect[i] == sym_section)
        {
          /* Yes... */

          if (verbose)
            {
              printf("Sym section %s is BSS\n", sym_section->name);
            }

          if (sym_segment)
            *sym_segment = &bss_info;
          return NXFLAT_RELOC_TYPE_BSS;
        }
    }

  /* Check if the symbol is defined in a TEXT section */

  for (i = 0; i < text_info.nsubsects; i++)
    {
      if (text_info.subsect[i] == sym_section)
        {
          /* Yes... */

          if (verbose)
            {
              printf("Sym section %s is CODE\n", sym_section->name);
            }

          if (sym_segment)
            *sym_segment = &text_info;
          return NXFLAT_RELOC_TYPE_TEXT;
        }
    }

  /* Check if the symbol is defined in a DATA section */

  for (i = 0; i < data_info.nsubsects; i++)
    {
      if (data_info.subsect[i] == sym_section)
        {
          /* Yes... */

          if (verbose)
            {
              printf("Sym section %s is DATA\n", sym_section->name);
            }

          if (sym_segment)
            *sym_segment = &data_info;
          return NXFLAT_RELOC_TYPE_DATA;
        }
    }

  fprintf(stderr,
          "ERROR: Could not find region for sym_section \"%s\" (%p)\n",
          sym_section->name, sym_section);
  return NXFLAT_RELOC_TYPE_NONE;
}

/***********************************************************************
 * resolve_segment_relocs
 ***********************************************************************/

static void
resolve_segment_relocs(bfd * input_bfd, segment_info * inf, asymbol ** syms,
                       u_int32_t * n_relocs, struct nxflat_reloc_s **relocs)
{
  struct nxflat_reloc_s *nxflat_relocs;
  arelent **relpp;
  u_int32_t nxflat_reloc_count;
  int relsize;
  int relcount;
  int i;
  int j;

  nxflat_relocs = *relocs;
  nxflat_reloc_count = *n_relocs;

  for (i = 0; i < inf->nsubsects; i++)
    {
      relcount = inf->subsect[i]->reloc_count;
      if (verbose)
        {
          printf("Section %s has %08x relocs\n",
                 inf->subsect[i]->name, relcount);
        }
      if (0 >= relcount)
        continue;

      relsize = bfd_get_reloc_upper_bound(input_bfd, inf->subsect[i]);
      if (verbose)
        {
          printf("Section %s reloc size: %08x\n",
                 inf->subsect[i]->name, relsize);
        }
      if (0 >= relsize)
        continue;

      relpp = (arelent **) malloc((size_t) relsize);
      relcount =
        bfd_canonicalize_reloc(input_bfd, inf->subsect[i], relpp, syms);

      if (relcount < 0)
        {
          fprintf(stderr, "ERROR: bfd_canonicalize_reloc failed!\n");
          exit(1);
        }

      if (verbose)
        {
          printf("Section %s can'd %08x relocs\n",
                 inf->subsect[i]->name, relcount);
        }

      for (j = 0; j < relcount; j++)
        {
          /* Get information about this symbol */

          reloc_howto_type *how_to      = relpp[j]->howto;
          asymbol          *rel_sym     = *relpp[j]->sym_ptr_ptr;
          asection         *rel_section = rel_sym->section;
          symvalue          sym_value;

          /* If the symbol is a thumb function, then set bit 1 of the value */

         sym_value = rel_sym->value;
#ifdef NXFLAT_ARM
         if ((((elf_symbol_type *)rel_sym)->internal_elf_sym.st_info & 0x0f) == STT_ARM_TFUNC)
            {
              sym_value |= 1;
            }
#endif

          if (verbose > 1)
            {
              printf("rel %d -> sym @ %p [%28s] s_addr @ %08x, "
                     "rel %08x how %s\n",
                     j, relpp[j]->sym_ptr_ptr, rel_sym->name,
                     (u_int32_t) relpp[j]->address,
                     (u_int32_t) relpp[j]->addend, how_to->name);
            }

          switch (how_to->type)
            {
            case R_ARM_PLT32:
            case R_ARM_PC24:
              {
                int32_t *opcode;
                int32_t temp;
                int32_t saved;

                if (verbose)
                  {
                    printf("performing   PC24 link at addr %08x to "
                           "sym [%20s] @ %08x\n",
                           (u_int32_t) relpp[j]->address, rel_sym->name,
                           (u_int32_t) sym_value);
                  }

                /* Can't fix what we ain't got */

                if (!(SEC_IN_MEMORY & rel_sym->section->flags))
                  {
                    fprintf(stderr, "ERROR: section %s not loaded into mem!\n",
                            rel_sym->section->name);
                    exit(1);
                  }

                /* PC24 -> can only fix text to text refs */

                if (!(SEC_CODE & rel_sym->section->flags))
                  {
                    fprintf(stderr, "ERROR: section %s not code!\n",
                            rel_sym->section->name);
                    exit(1);
                  }

                if (!(SEC_CODE & inf->subsect[i]->flags))
                  {
                    fprintf(stderr, "ERROR: section %s not code!\n",
                            rel_sym->section->name);
                    exit(1);
                  }

                opcode = (int32_t *) (inf->contents + relpp[j]->address);
                if (verbose > 1)
                  {
                    printf("original opcode @ %p is %08x ",
#ifdef ARCH_BIG_ENDIAN
                           opcode, (int32_t) nxflat_swap32(*opcode));
#else
                           opcode, *opcode);
#endif
                    printf("rsh %d ", how_to->rightshift);
                    printf(" sz %d ", how_to->size);
                    printf("bit %d ", how_to->bitsize);
                    printf("rel %d ", how_to->pc_relative);
                    printf("smask %08x ", (u_int32_t) how_to->src_mask);
                    printf("dmask %08x ", (u_int32_t) how_to->dst_mask);
                    printf("off %d ", how_to->pcrel_offset);
                    printf("\n");
                  }

                if (how_to->pcrel_offset)
                  {
#ifdef ARCH_BIG_ENDIAN
                    saved = temp = (int32_t) nxflat_swap32(*opcode);
#else
                    saved = temp = *opcode;
#endif
                    /* mask */
                    temp &= how_to->src_mask;

                    /* sign extend */
                    temp <<= (32 - how_to->bitsize);
                    temp >>= (32 - how_to->bitsize);

                    /* offset */
                    temp +=
                      ((sym_value + rel_sym->section->vma)
                       - relpp[j]->address) >> how_to->rightshift;

                    /* demote */
                    /* temp >>= how_to->rightshift; */

                    /* mask upper bits from rollover */
                    temp &= how_to->dst_mask;

                    /* replace data that was masked */
                    temp |= saved & (~how_to->dst_mask);
                  }
                else
                  {
                    fprintf(stderr, "ERROR: Do not know how pcrel_offset\n");
                    exit(1);
                  }

                if (verbose > 1)
                  {
                    printf("result opcode: %08x\n", temp);
                  }
#ifdef ARCH_BIG_ENDIAN
                *opcode = (int32_t) nxflat_swap32(temp);
#else
                *opcode = temp;
#endif
              }
              break;

            case R_ARM_ABS32:
              {
                int32_t *opcode;
                int32_t temp;
                int32_t saved;

                if (verbose)
                  {
                    printf("performing ABS32 link at addr %08x to "
                           "sym [%20s] @ %08x\n",
                           (u_int32_t) relpp[j]->address, rel_sym->name,
                           (u_int32_t) sym_value);
                  }

                /* ABS32 links from .text are easy - since the fetches will */
                /* always be base relative. the ABS32 refs from data will be */
                /* handled the same, with an appropriate reloc record for */
                /* the load_flat_binary() kernel routine to handle */

                opcode = (int32_t *) (inf->contents + relpp[j]->address);
                if (verbose)
                  {
                    printf("original opcode @ %p is %08x ",
#ifdef ARCH_BIG_ENDIAN
                           opcode, (int32_t) nxflat_swap32(*opcode));
#else
                           opcode, *opcode);
#endif
                    printf("rsh %d ", how_to->rightshift);
                    printf(" sz %d ", how_to->size);
                    printf("bit %d ", how_to->bitsize);
                    printf("rel %d ", how_to->pc_relative);
                    printf("smask %08x ", (u_int32_t) how_to->src_mask);
                    printf("dmask %08x ", (u_int32_t) how_to->dst_mask);
                    printf("off %d ", how_to->pcrel_offset);
                    printf("\n");
                  }

#ifdef ARCH_BIG_ENDIAN
                saved = temp = (int32_t) nxflat_swap32(*opcode);
#else
                saved = temp = *opcode;
#endif
                /* mask */

                temp &= how_to->src_mask;

                /* sign extend */

                temp <<= (32 - how_to->bitsize);
                temp >>= (32 - how_to->bitsize);

                /* offset */

                temp += (sym_value + rel_sym->section->vma) >>
                  how_to->rightshift;

                /* demote */
                /* temp >>= how_to->rightshift; */

                /* mask upper bits from rollover */

                temp &= how_to->dst_mask;

                /* replace data that was masked */

                temp |= saved & (~how_to->dst_mask);

                if (verbose)
                  {
                    printf("result opcode: %08x\n", temp);
                  }
#ifdef ARCH_BIG_ENDIAN
                *opcode = (int32_t) nxflat_swap32(temp);
#else
                *opcode = temp;
#endif
                if ((inf->subsect[i]->flags & SEC_DATA) &&
                    (inf->subsect[i]->flags & SEC_ALLOC))
                  {
                    int reltype;

                    if (verbose)
                      {
                        printf("RELOCATION in DATA!\n");
                      }

                    /* Locate the address referred to by section type. */

                    reltype = get_reloc_type(rel_section, NULL);

                    /* Create space for one more relocation. */

                    nxflat_relocs = realloc(nxflat_relocs,
                                           (nxflat_reloc_count + 1)
                                           * sizeof(struct nxflat_reloc_s));

                    /* And tuck in the new relocation. */

                    
                    nxflat_relocs[nxflat_reloc_count].r_info = NXFLAT_RELOC(reltype, relpp[j]->address);
                    nxflat_reloc_count++;
                  }
              }
              break;

            case R_ARM_GOTOFF:
              /* Relocation is relative to the start of the global offset
               * table. */

              fprintf(stderr,
                      "Attempted  GOTOFF reloc at addr %08x to "
                      "sym [%20s] @ %08x\n",
                      (u_int32_t) relpp[j]->address, rel_sym->name,
                      (u_int32_t) sym_value);
              goto got_not_supported;

            case R_ARM_GOT32:
              /* Relocation is to the entry for this symbol in the global
               * offset table. */
              fprintf(stderr,
                      "Attempted  GOT32 reloc at addr %08x to "
                      "sym [%20s] @ %08x\n",
                      (u_int32_t) relpp[j]->address, rel_sym->name,
                      (u_int32_t) sym_value);
              goto got_not_supported;

            case R_ARM_GOTPC:
              /* Use global offset table as symbol value.  */
              fprintf(stderr,
                      "Attempted  GOTPC reloc at addr %08x to "
                      "sym [%20s] @ %08x\n",
                      (u_int32_t) relpp[j]->address, rel_sym->name,
                      (u_int32_t) sym_value);

            got_not_supported:
              fprintf(stderr,
                      "You must compile using the -mno-got or -membedded-pic "
                      "compiler option\n");
              break;

            default:
              fprintf(stderr,
                      "Do not know how to handle reloc %d type %s @ %p!\n",
                      how_to->type, how_to->name, how_to);
              break;
            }
        }

      /* Mark the section as having no relocs */

      inf->subsect[i]->flags &= !(SEC_RELOC);
      inf->subsect[i]->reloc_count = 0;
    }

  /* Write reloc record data back out */

  *relocs = nxflat_relocs;
  *n_relocs = nxflat_reloc_count;
}

/***********************************************************************
 * dump_symbol
 ***********************************************************************/

static void dump_symbol(asymbol * psym)
{
  struct elf_internal_sym *isym =
    (struct elf_internal_sym *)&((elf_symbol_type *)psym)->internal_elf_sym;

  if (bfd_is_com_section(psym->section))
    {
      /* Common Global - unplaced */

      printf("Sym[%24s] @            sz %04x ",
             psym->name, (u_int32_t) psym->value);
      printf("align %04x ", (u_int32_t)isym->st_value);
    }
  else
    {
      printf("Sym[%24s] @ %04x align            ",
             psym->name, (u_int32_t) psym->value);
      printf("sz %04x ", (u_int32_t)isym->st_size);
    }

  /* Symbol type */

  printf("tp %02x ", isym->st_info);

  /* Tag thumb specific attributes */

#ifdef NXFLAT_ARM
  if ((isym->st_info & 0x0f) == STT_ARM_TFUNC || (isym->st_info & 0x0f) == STT_ARM_16BIT)
    {
      putchar('T');
    }
  else
    {
      putchar(' ');
    }
#endif

  /* Common attributes */

  printf("|%c", psym->flags & BSF_OBJECT ? 'O' : '.');
  printf("%c",  psym->flags & BSF_DYNAMIC ? 'D' : '.');
  printf("%c",  psym->flags & BSF_FILE ? 'F' : '.');
  printf("%c",  psym->flags & BSF_INDIRECT ? 'I' : '.');
  printf("%c",  psym->flags & BSF_WARNING ? 'W' : '.');
  printf("%c",  psym->flags & BSF_CONSTRUCTOR ? 'C' : '.');
  printf("%c",  psym->flags & BSF_NOT_AT_END ? 'N' : '.');
  printf("%c",  psym->flags & BSF_OLD_COMMON ? 'c' : '.');
  printf("%c",  psym->flags & BSF_SECTION_SYM ? 'S' : '.');
  printf("%c",  psym->flags & BSF_WEAK ? 'w' : '.');
  printf("%c",  psym->flags & BSF_KEEP_G ? 'G' : '.');
  printf("%c",  psym->flags & BSF_KEEP ? 'K' : '.');
  printf("%c",  psym->flags & BSF_FUNCTION ? 'f' : '.');
  printf("%c",  psym->flags & BSF_DEBUGGING ? 'd' : '.');
  printf("%c",  psym->flags & BSF_GLOBAL ? 'g' : '.');
  printf("%c|", psym->flags & BSF_LOCAL ? 'l' : '.');
  printf("\n");
}

/***********************************************************************
 * check_symbol_overlap
 ***********************************************************************/

static void check_symbol_overlap(asymbol ** symbols, int number_of_symbols)
{
  int i;
  int j;

  for (i = 0; i < number_of_symbols; i++)
    {
      elf_symbol_type *sym_i;
      bfd_vma base_i;
      bfd_vma top_i;
      bfd_vma size_i;

      sym_i = (elf_symbol_type *) symbols[i];
      base_i = sym_i->symbol.section->vma + sym_i->internal_elf_sym.st_value;
      size_i = sym_i->internal_elf_sym.st_size;

      if (0 == size_i)
        {
          if (sym_i->symbol.section->flags & SEC_CODE)
            {
              /* must be an internal branch - ignore */
              if (verbose)
                printf("Sym [%20s] is zero len, skipping!\n",
                       sym_i->symbol.name);
              continue;
            }
          else
            {
              /* pointer - fake size up */
              size_i = 4;
            }
        }

      top_i = base_i + size_i;

      if (verbose)
        {
          printf("Sym->[%20s] base %08x, top %08x\n",
                 sym_i->symbol.name, (u_int32_t) base_i, (u_int32_t) top_i);
        }

      for (j = (i + 1); j < number_of_symbols; j++)
        {
          elf_symbol_type *sym_j;
          bfd_vma base_j;
          bfd_vma top_j;
          bfd_vma size_j = 0;

          sym_j = (elf_symbol_type *) symbols[j];
          base_j =
            sym_j->symbol.section->vma + sym_j->internal_elf_sym.st_value;

          if (0 == size_j)
            {
              if (sym_j->symbol.section->flags & SEC_CODE)
                {
                  /* must be an internal branch - ignore */
                  continue;
                }
              else
                {
                  /* pointer - fake size up */
                  size_j = 4;
                }
            }

          top_j = base_j + sym_j->internal_elf_sym.st_size;

          if (0 == sym_j->internal_elf_sym.st_size)
            {
              continue;
            }

          if ((base_j < top_i) && (top_j > base_i))
            {
              /* symbols overlap - bad bad bad bad */

              if (verbose)
                {
                  fprintf(stdout,
                          "symbols [%20s][%6s] and [%20s][%6s] OVERLAP!\n",
                          sym_i->symbol.name, sym_i->symbol.section->name,
                          sym_j->symbol.name, sym_j->symbol.section->name);
                  fprintf(stdout, "Sym->[%20s] base %08x, top %08x\n",
                          sym_i->symbol.name, (u_int32_t) base_i,
                          (u_int32_t) top_i);
                  fprintf(stdout, "Sym->[%20s] base %08x, top %08x\n",
                          sym_j->symbol.name, (u_int32_t) base_j,
                          (u_int32_t) top_j);
                }
            }

        }
    }
}

/***********************************************************************
 * map_common_symbols
 ***********************************************************************/

static void
map_common_symbols(bfd * input_bfd, asymbol ** symbols, int number_of_symbols)
{
  asection *bss_s;
  int i;
  int j;

  bfd_vma baseaddr;
  bfd_vma align;
  bfd_vma size;
  bfd_vma symbase;
  bfd_vma offset;

  bss_s = bss_info.subsect[0];
  baseaddr = 0;

  printf("checking overlap before mapping\n");

  check_symbol_overlap(symbols, number_of_symbols);

  printf("mapping COMMONS\n");

  if (NULL == bss_s)
    {
      fprintf(stderr, "WARNING: NULL section passed to map_common_symbols\n");
      return;
    }

  if (verbose)
    {
      printf("Assigning COMMON symbols to section %s\n", bss_s->name);
    }

  for (i = 0; i < number_of_symbols; i++)
    {
      if (bfd_is_com_section(symbols[i]->section))
        {
          if (verbose > 1)
            {
              printf("COMMON sym[%04d] -> ", i);
              dump_symbol(symbols[i]);
            }

          /* get parameters of unmapped symbol */
#if 0
          align = ((elf_symbol_type *) symbols[i])->internal_elf_sym.st_value;
#else
          /* Ignore alignment - just make sure we're word aligned we're not
           * worrying about page boundaries since we're flat mem and we're not
           * really concerned with cache alignment - maybe someday */

          align = 0x04;
#endif
          size = ((elf_symbol_type *) symbols[i])->internal_elf_sym.st_size;

          if (0 == size)
            {
              if (verbose)
                {
                  printf("zero size symbol assumed to be a ptr size = 4\n");
                }
              size = 0x04;
            }

          if (size % 0x04)
            {
              printf("non-mod4 symbol rounded up 4\n");
              size = ((size >> 2) + 1) << 2;
            }

          /* INSERT SYMBOL AT END OF BSS - MUCH MO BETTA */

          /* calulate transaction effects - insert blank b4 sym to get align */

          baseaddr = bss_s->COOKED_SIZE;
          symbase = ((baseaddr + align - 1) / align) * align;
          offset = (symbase + size) - baseaddr;

          if (verbose > 1)
            {
              printf(" ba: %08x sb: %08x al: %04x sz: %04x of: %04x\n",
                     (u_int32_t) baseaddr, (u_int32_t) symbase,
                     (u_int32_t) align, (u_int32_t) size, (u_int32_t) offset);
            }

          /* Add space to bss segment and section */

          bss_info.high_mark += offset;
          bss_info.size += offset;
          bss_s->COOKED_SIZE += offset;
          bss_s->RAW_SIZE += offset;

          /* find all end markers and offset */

          for (j = 0; j < number_of_symbols; j++)
            {
              if (bss_s == symbols[j]->section)
                {
                  if (verbose > 1)
                    {
                      printf("checking endsym? %08x sym[%04d] -> ",
                             (u_int32_t) baseaddr, j);
                      dump_symbol(symbols[j]);
                    }

                  if (symbols[j]->value >= baseaddr)
                    {
                      symbols[j]->value += offset;
                      ((elf_symbol_type *) symbols[j])->internal_elf_sym.
                        st_value += offset;
                      if (verbose > 1)
                        {
                          printf("sym MOVED!!!\n");
                          printf("to sym[%04d] -> ", j);
                          dump_symbol(symbols[j]);
                        }
                    }
                }
            }

          /* stuff sym at base */

          symbols[i]->section = bss_s;
          symbols[i]->value = symbase;
          symbols[i]->flags = BSF_OBJECT | BSF_GLOBAL;
          ((elf_symbol_type *) symbols[i])->internal_elf_sym.st_value = symbase;

          if (verbose > 1)
            {
              printf("NEW sym[%04d] -> ", i);
              dump_symbol(symbols[i]);
            }
        }
    }

  check_symbol_overlap(symbols, number_of_symbols);
}

/***********************************************************************
 * output_relocs
 ***********************************************************************/

static struct nxflat_reloc_s *output_relocs(bfd * input_bfd, asymbol ** symbols,
                                            int number_of_symbols,
                                            u_int32_t * n_relocs)
{
  struct nxflat_reloc_s *nxflat_relocs;
  u_int32_t nxflat_reloc_count;
  int rc;
  int i;

  *n_relocs = 0;
  nxflat_relocs = NULL;
  nxflat_reloc_count = 0;
  rc = 0;

  if (verbose)
    {
      printf(" Before mapp high mark %08x cooked %08x raw %08x \n",
             (u_int32_t) bss_info.high_mark,
             (u_int32_t) bss_info.subsect[0]->COOKED_SIZE,
             (u_int32_t) bss_info.subsect[0]->RAW_SIZE);
    }

  /* Unmapped 'common' symbols need to be stuffed into bss */

  map_common_symbols(input_bfd, symbols, number_of_symbols);

  if (verbose)
    {
      printf(" After map high mark %08x cooked %08x raw %08x \n",
             (u_int32_t) bss_info.high_mark,
             (u_int32_t) bss_info.subsect[0]->COOKED_SIZE,
             (u_int32_t) bss_info.subsect[0]->RAW_SIZE);
    }

  if (verbose)
    {
      for (i = 0; i < number_of_symbols; i++)
        {
          printf("sym[%04d] -> ", i);
          dump_symbol(symbols[i]);
        }
    }

  /* Stuff the addrs into the code (and data) */

  resolve_segment_relocs(input_bfd, &text_info, symbols,
                         &nxflat_reloc_count, &nxflat_relocs);
  resolve_segment_relocs(input_bfd, &data_info, symbols,
                         &nxflat_reloc_count, &nxflat_relocs);
  resolve_segment_relocs(input_bfd, &bss_info, symbols,
                         &nxflat_reloc_count, &nxflat_relocs);

  *n_relocs = nxflat_reloc_count;

  printf(" returning %d relocs\n", nxflat_reloc_count);

  /* need to emit relocs for data only */
  return nxflat_relocs;
}

/***********************************************************************
 * is_unwanted_section
 ***********************************************************************/

/* Return 1 if this is a section that we want to throw away but can only
 * identify by name.   Normally, any section with appropriate-looking flags
 * will get copied into the output file.
 */

static int is_unwanted_section(asection * s)
{
  if (!strcmp(s->name, ".hash"))
    return 1;
  if (!strcmp(s->name, ".dynstr") || !strcmp(s->name, ".dynsym"))
    return 1;
  if (s->flags & SEC_DEBUGGING)
    return 1;
  return 0;
}

/***********************************************************************
 * register_section
 ***********************************************************************/

/* Mark this section for inclusion in some segment.  */
static void register_section(asection * s, segment_info * inf)
{
  if (verbose)
    printf("registering section %s to %s segment\n", s->name, inf->name);
  inf->subsect[inf->nsubsects++] = s;
}

/***********************************************************************
 * dump_sections
 ***********************************************************************/

/* Print out the sections that make up this segment for debugging.  */
static void dump_sections(segment_info * inf)
{
  int i;
  printf("      [ ");
  for (i = 0; i < inf->nsubsects; i++)
    printf("%s ", inf->subsect[i]->name);
  printf("]\n");
}

/***********************************************************************
 * load_sections
 ***********************************************************************/

/* Pull the sections that make up this segment in off disk.  */
static void load_sections(bfd * bfd, segment_info * inf)
{
  int i;
  void *ptr;
  if (!inf->size)
    return;                     /* Nothing to do */
  inf->contents = malloc(inf->size);
  if (!inf->contents)
    {
      fprintf(stderr, "ERROR: Out of memory.\n");
      exit(1);
    }
  ptr = inf->contents;
  for (i = 0; i < inf->nsubsects; i++)
    {
      if (!bfd_get_section_contents(bfd, inf->subsect[i], ptr,
                                    0, inf->subsect[i]->COOKED_SIZE))
        {
          fprintf(stderr, "ERROR: Failed to read section contents.\n");
          exit(1);
        }
      ptr += inf->subsect[i]->COOKED_SIZE;
      inf->subsect[i]->flags |= SEC_IN_MEMORY;
    }
}

/***********************************************************************
 * stack_nxflat_segment
 ***********************************************************************/

static void stack_nxflat_segment(segment_info * inf)
{
  bfd_vma min_addr = 0x7fffffff;
  bfd_vma max_addr = 0x00000000;
  int i;

  for (i = 0; i < inf->nsubsects; i++)
    {
      bfd_vma vma = inf->subsect[i]->vma;
      if (vma < min_addr)
        min_addr = vma;

      vma += inf->subsect[i]->COOKED_SIZE;
      if (vma > max_addr)
        max_addr = vma;
    }

  inf->low_mark = min_addr;
  inf->high_mark = max_addr;
  inf->size = max_addr - min_addr;
}

/***********************************************************************
 * show_usage
 ***********************************************************************/

static void show_usage(void)
{
  fprintf(stderr, "Usage: %s [options] <bfd-filename>\n\n", program_name);
  fprintf(stderr, "Where options are one or more of the following.  Note\n");
  fprintf(stderr, "that a space is always required between the option and\n");
  fprintf(stderr, "any following arguments.\n\n");
  fprintf(stderr, "  -d Use dynamic symbol table [symtab]\n");
  fprintf(stderr, "  -e <entry-point>\n");
  fprintf(stderr, "     Entry point to module [%s for executable]\n",
          default_exe_entry_name);
  fprintf(stderr, "     NULL for shared library\n");
  fprintf(stderr, "  -o <out-filename>\n");
  fprintf(stderr, "     Output to <out-filename> [<bfd-filename>.xflt]\n");
  fprintf(stderr, "  -s <stack-size>\n");
  fprintf(stderr, "     Set stack size to <stack-size>.  Ignored if -l also\n");
  fprintf(stderr, "     selected. [%d]\n", DEFAULT_STACK_SIZE);
  fprintf(stderr, "  -v Verbose output [no verbose output]\n");
  fprintf(stderr, "     If -v is applied twice, additional debug output\n");
  fprintf(stderr, "     be enabled.\n");
  fprintf(stderr, "\n");
  exit(2);
}

/***********************************************************************
 * parse_args
 ***********************************************************************/

static void parse_args(int argc, char **argv)
{
  int opt;

  /* Save our name (for show_usage) */

  program_name = argv[0];

  /* Set some default values */

  stack_size = 0;
  entry_name = NULL;

  if (argc < 2)
    {
      fprintf(stderr, "ERROR: Missing required arguments\n\n");
      show_usage();
    }

  /* Get miscellaneous options from the command line. */

  while ((opt = getopt(argc, argv, "de:lo:s:v")) != -1)
    {
      switch (opt)
        {

        case 'd':
          dsyms++;
          break;

        case 'e':
          entry_name = optarg;
          break;

        case 'o':
          out_filename = optarg;
          break;

        case 's':
          stack_size = atoi(optarg);
          break;

        case 'v':
          verbose++;
          break;

        case 'l':
        default:
          fprintf(stderr, "ERROR: %s Unknown option\n\n", argv[0]);
          show_usage();
          break;
        }
    }

  /* The very last thing is also the name of the BFD input file */

  bfd_filename = argv[argc - 1];

  /* Verify that an appropriate stack size is selected. */

  if (stack_size == 0)
    {
      /* Executables must have a stack_size selected. */

      printf("Using default stack size: %d\n", DEFAULT_STACK_SIZE);
      stack_size = DEFAULT_STACK_SIZE;
    }

  if (entry_name == NULL)
    {
      printf("Using entry_point: %s\n", default_exe_entry_name);
      entry_name = default_exe_entry_name;
    }
}

/***********************************************************************
 * Public Functions
 ***********************************************************************/

/***********************************************************************
 * main
 ***********************************************************************/

int main(int argc, char **argv, char **envp)
{
  int fd;
  bfd *bf;
  asection *s;

  asymbol **symbol_table;
  int32_t number_of_symbols;

  u_int32_t reloc_len;
  u_int32_t *reloc;

  struct nxflat_hdr_s hdr;

  /* Parse the incoming command line */

  parse_args(argc, argv);

  /* Open the BFD input file */

  if (!(bf = bfd_openr(argv[argc - 1], 0)))
    {
      fprintf(stderr, "ERROR: Failed to open %s\n", argv[argc - 1]);
      exit(1);
    }

  /* Verify the format of the BFD file */

  if (bfd_check_format(bf, bfd_object) == 0)
    {
      printf("File is not an object file\n");
      exit(2);
    }

  /* Read the symbol table from the file */

  symbol_table = get_symbols(bf, &number_of_symbols);

  /* Find all of the special symbols that we will need in the symbol table that 
   * we just read. */

  find_special_symbols();

  /* Walk the list of sections, figuring out where each one goes and how much
   * storage it requires. */

  text_info.low_mark = data_info.low_mark = bss_info.low_mark = -1;
  text_info.high_mark = data_info.high_mark = bss_info.high_mark = 0;
  text_info.contents = data_info.contents = bss_info.contents = NULL;
  text_info.size = data_info.size = bss_info.size = 0;
  text_info.nsubsects = data_info.nsubsects = bss_info.nsubsects = 0;
  text_info.name = "text";
  data_info.name = "data";
  bss_info.name = "bss";

  for (s = bf->sections; s != NULL; s = s->next)
    {
      if (verbose)
        printf("reading section %s\n", s->name);

      /* ignore blatantly useless sections */
      if (is_unwanted_section(s))
        continue;

      if (s->flags == SEC_ALLOC)
        {
          if (verbose)
            {
              printf("Section %s is ALLOC only\n", s->name);
            }
          register_section(s, &bss_info);
          continue;
        }
      if ((s->flags & SEC_CODE) && (s->flags & SEC_ALLOC))
        {
          if (verbose)
            {
              printf("Section %s is CODE\n", s->name);
            }
          register_section(s, &text_info);
          continue;
        }
      if ((s->flags & SEC_DATA) && (s->flags & SEC_ALLOC))
        {
          if (verbose)
            {
              printf("section %s is DATA\n", s->name);
            }
          register_section(s, &data_info);
          continue;
        }
      if (verbose)
        printf("WARNING: ignoring section %s\n", s->name);
    }

  /* Fixup high and low water VMA address */

  stack_nxflat_segment(&text_info);
  stack_nxflat_segment(&data_info);
  stack_nxflat_segment(&bss_info);

  /* Check for a data offset due to the presence of a GOT */

  printf("INPUT SECTIONS:\n");
  printf("SECT LOW MARK   HIGH MARK  (SIZE BYTES)\n");
  if (text_info.nsubsects == 0)
    {
      printf("TEXT Not found  Not found ( Not found )\n");
    }
  else
    {
      printf("TEXT %08x %08x (%08lx)\n",
             (u_int32_t) text_info.low_mark, (u_int32_t) text_info.high_mark,
             text_info.size);

      if (text_info.low_mark != 0)
        {
          fprintf(stderr, "ERROR: text section must be origined at zero");
          exit(1);
        }
    }

  if (data_info.nsubsects == 0)
    {
      printf("DATA Not found  Not found ( Not found )\n");
    }
  else
    {
      printf("DATA %08x %08x (%08lx)\n",
             (u_int32_t) data_info.low_mark, (u_int32_t) data_info.high_mark,
             data_info.size);

      if (data_info.low_mark != 0)
        {
          fprintf(stderr, "ERROR: data section must be origined at zero");
          exit(1);
        }
    }

  if (bss_info.nsubsects == 0)
    {
      printf("BSS  Not found  Not found ( Not found )\n");
    }
  else
    {
      printf("BSS  %08x %08x (%08lx)\n",
             (u_int32_t) bss_info.low_mark, (u_int32_t) bss_info.high_mark,
             bss_info.size);

      /* If data is present, then BSS was be origined immediately after the
       * data. */

      if (data_info.nsubsects > 0)
        {
          /* There is data... Account for possible ALIGN 0x10 at end of data */

          u_int32_t bss_start1 = data_info.high_mark;
          u_int32_t bss_start2 = ((bss_start1 + 0x0f) & ~0x0f);

          if ((bss_info.low_mark < bss_start1) &&
              (bss_info.low_mark > bss_start2))
            {
              fprintf(stderr,
                      "ERROR: bss must be origined immediately after "
                      "the data section\n");
              exit(1);
            }
        }

      /* If there is no data, then the BSS must be origined at zero */

      else if (bss_info.low_mark != 0)
        {
          fprintf(stderr, "ERROR: bss section (with no data section) must be");
          fprintf(stderr, "       origined at zero\n");
          exit(1);
        }
    }

  if (verbose)
    {
      printf("TEXT:");
      dump_sections(&text_info);
      printf("DATA:");
      dump_sections(&data_info);
      printf("BSS: ");
      dump_sections(&bss_info);
    }

  /* Slurp the section contents in.  No need to load BSS since we know * it
   * isn't initialised. */

  load_sections(bf, &text_info);
  load_sections(bf, &data_info);

  reloc = (u_int32_t*)output_relocs(bf, symbol_table, number_of_symbols, &reloc_len);

  /* Fill in the xFLT file header */

  memcpy(hdr.h_magic, "xFLT", 4);

  put_xflat32(&hdr.h_datastart, NXFLAT_HDR_SIZE + text_info.size);
  put_xflat32(&hdr.h_dataend, NXFLAT_HDR_SIZE + text_info.size + data_info.size);
  put_xflat32(&hdr.h_bssend,
              NXFLAT_HDR_SIZE + text_info.size + data_info.size +
              bss_info.size);
  put_xflat32(&hdr.h_stacksize, stack_size);     /* FIXME */
  put_xflat32(&hdr.h_relocstart,
              NXFLAT_HDR_SIZE + text_info.size + data_info.size);
  put_xflat32(&hdr.h_reloccount, reloc_len);

  put_entry_point(&hdr);

  put_special_symbol(dynimport_begin_symbol,
                     dynimport_end_symbol,
                     &hdr.h_importsymbols,
                     &hdr.h_importcount,
                     sizeof(struct nxflat_import_s), text_info.size);

 #ifdef RELOCS_IN_NETWORK_ORDER
  {
    int i;
    for (i = 0; i < reloc_len; i++)
      {
        reloc[i] = htonl(reloc[i]);
      }
  }
#endif

  if (verbose && reloc)
    {
      printf("reloc size: %04x\n", reloc_len);
    }

  if (!out_filename)
    {
      out_filename = malloc(strlen(bfd_filename) + 5);  /* 5 to add suffix */
      strcpy(out_filename, bfd_filename);
      strcat(out_filename, ".xflt");
    }

  fd = open(out_filename, O_WRONLY | O_PLATFORM | O_CREAT | O_TRUNC, 0744);
  if (fd < 0)
    {
      fprintf(stderr, "ERROR: Failed open output file %s\n", out_filename);
      exit(4);
    }

  write(fd, &hdr, sizeof(hdr));
  write(fd, text_info.contents, text_info.size);
  write(fd, data_info.contents, data_info.size);

  if (reloc)
    {
      write(fd, reloc, reloc_len * 4);
    }

  close(fd);

  exit(0);
}
