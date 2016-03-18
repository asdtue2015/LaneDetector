/*
  File autogenerated by gengetopt version 2.18
  generated with the following command:
  gengetopt -i cameraInfoOpt.ggo -F cameraInfoOpt --func-name=cameraInfoParser --arg-struct-name=CameraInfoParserInfo --conf-parser

  The developers of gengetopt consider the fixed text that goes in all
  gengetopt output files to be in the public domain:
  we make no copyright claims on it.
*/

/* If we use autoconf.  */
#ifdef HAVE_CONFIG_H
extern "C" {
#include "config.h"
}
#endif

extern "C" {
#include <stdio.h>
}

extern "C" {
#include <stdlib.h>
}

extern "C" {
#include <string.h>
}

extern "C" {
#include "getopt.h"
}

extern "C" {
#include "CameraInfoOpt.h"
}

const char *CameraInfoParserInfo_purpose = "";

const char *CameraInfoParserInfo_usage = "Usage: inversePerspectiveMapping [OPTIONS]...";

const char *CameraInfoParserInfo_help[] = {
  "  -h, --help                   Print help and exit",
  "  -V, --version                Print version and exit",
  "      --focalLengthX=DOUBLE    Focal lenght in horizontal direction in pixels",
  "      --focalLengthY=DOUBLE    Focal lenght in vertical direction in pixels",
  "      --opticalCenterX=DOUBLE  X-coordinate of optical center in pixels",
  "      --opticalCenterY=DOUBLE  Y-coordinate of optical center in pixels",
  "      --cameraHeight=DOUBLE    Height of camera above ground in mm",
  "      --pitch=DOUBLE           pitch of camera in degrees (+ve downwards)",
  "      --yaw=DOUBLE             yaw of camera in degrees (+ve clockwise)",
  "      --imageWidth=DOUBLE      width of image in pixels",
  "      --imageHeight=DOUBLE     height of image in pixels",
    0
};

static
void clear_given (struct CameraInfoParserInfo *args_info);
static
void clear_args (struct CameraInfoParserInfo *args_info);

static int
cameraInfoParser_internal (int argc, char * const *argv, struct CameraInfoParserInfo *args_info, int override, int initialize, int check_required, const char *additional_error);

static int
cameraInfoParser_required2 (struct CameraInfoParserInfo *args_info, const char *prog_name, const char *additional_error);
struct line_list
{
  char * string_arg;
  struct line_list * next;
};

static struct line_list *cmd_line_list = 0;
static struct line_list *cmd_line_list_tmp = 0;

static void
free_cmd_list(void)
{
  /* free the list of a previous call */
  if (cmd_line_list)
    {
      while (cmd_line_list) {
        cmd_line_list_tmp = cmd_line_list;
        cmd_line_list = cmd_line_list->next;
        free (cmd_line_list_tmp->string_arg);
        free (cmd_line_list_tmp);
      }
    }
}


static char *
gengetopt_strdup (const char *s);

static
void clear_given (struct CameraInfoParserInfo *args_info)
{
  args_info->help_given = 0 ;
  args_info->version_given = 0 ;
  args_info->focalLengthX_given = 0 ;
  args_info->focalLengthY_given = 0 ;
  args_info->opticalCenterX_given = 0 ;
  args_info->opticalCenterY_given = 0 ;
  args_info->cameraHeight_given = 0 ;
  args_info->pitch_given = 0 ;
  args_info->yaw_given = 0 ;
  args_info->imageWidth_given = 0 ;
  args_info->imageHeight_given = 0 ;
}

static
void clear_args (struct CameraInfoParserInfo *args_info)
{
  args_info->focalLengthX_orig = NULL;
  args_info->focalLengthY_orig = NULL;
  args_info->opticalCenterX_orig = NULL;
  args_info->opticalCenterY_orig = NULL;
  args_info->cameraHeight_orig = NULL;
  args_info->pitch_orig = NULL;
  args_info->yaw_orig = NULL;
  args_info->imageWidth_orig = NULL;
  args_info->imageHeight_orig = NULL;

}

static
void init_args_info(struct CameraInfoParserInfo *args_info)
{
  args_info->help_help = CameraInfoParserInfo_help[0] ;
  args_info->version_help = CameraInfoParserInfo_help[1] ;
  args_info->focalLengthX_help = CameraInfoParserInfo_help[2] ;
  args_info->focalLengthY_help = CameraInfoParserInfo_help[3] ;
  args_info->opticalCenterX_help = CameraInfoParserInfo_help[4] ;
  args_info->opticalCenterY_help = CameraInfoParserInfo_help[5] ;
  args_info->cameraHeight_help = CameraInfoParserInfo_help[6] ;
  args_info->pitch_help = CameraInfoParserInfo_help[7] ;
  args_info->yaw_help = CameraInfoParserInfo_help[8] ;
  args_info->imageWidth_help = CameraInfoParserInfo_help[9] ;
  args_info->imageHeight_help = CameraInfoParserInfo_help[10] ;

}

void
cameraInfoParser_print_version (void)
{
  printf ("%s %s\n", CAMERAINFOPARSER_PACKAGE, CAMERAINFOPARSER_VERSION);
}

void
cameraInfoParser_print_help (void)
{
  int i = 0;
  cameraInfoParser_print_version ();

  if (strlen(CameraInfoParserInfo_purpose) > 0)
    printf("\n%s\n", CameraInfoParserInfo_purpose);

  printf("\n%s\n\n", CameraInfoParserInfo_usage);
  while (CameraInfoParserInfo_help[i])
    printf("%s\n", CameraInfoParserInfo_help[i++]);
}

void
cameraInfoParser_init (struct CameraInfoParserInfo *args_info)
{
  clear_given (args_info);
  clear_args (args_info);
  init_args_info (args_info);
}

static void
cameraInfoParser_release (struct CameraInfoParserInfo *args_info)
{

  if (args_info->focalLengthX_orig)
    {
      free (args_info->focalLengthX_orig); /* free previous argument */
      args_info->focalLengthX_orig = 0;
    }
  if (args_info->focalLengthY_orig)
    {
      free (args_info->focalLengthY_orig); /* free previous argument */
      args_info->focalLengthY_orig = 0;
    }
  if (args_info->opticalCenterX_orig)
    {
      free (args_info->opticalCenterX_orig); /* free previous argument */
      args_info->opticalCenterX_orig = 0;
    }
  if (args_info->opticalCenterY_orig)
    {
      free (args_info->opticalCenterY_orig); /* free previous argument */
      args_info->opticalCenterY_orig = 0;
    }
  if (args_info->cameraHeight_orig)
    {
      free (args_info->cameraHeight_orig); /* free previous argument */
      args_info->cameraHeight_orig = 0;
    }
  if (args_info->pitch_orig)
    {
      free (args_info->pitch_orig); /* free previous argument */
      args_info->pitch_orig = 0;
    }
  if (args_info->yaw_orig)
    {
      free (args_info->yaw_orig); /* free previous argument */
      args_info->yaw_orig = 0;
    }
  if (args_info->imageWidth_orig)
    {
      free (args_info->imageWidth_orig); /* free previous argument */
      args_info->imageWidth_orig = 0;
    }
  if (args_info->imageHeight_orig)
    {
      free (args_info->imageHeight_orig); /* free previous argument */
      args_info->imageHeight_orig = 0;
    }

  clear_given (args_info);
}

int
cameraInfoParser_file_save(const char *filename, struct CameraInfoParserInfo *args_info)
{
  FILE *outfile;
  int i = 0;

  outfile = fopen(filename, "w");

  if (!outfile)
    {
      fprintf (stderr, "%s: cannot open file for writing: %s\n", CAMERAINFOPARSER_PACKAGE, filename);
      return EXIT_FAILURE;
    }

  if (args_info->help_given) {
    fprintf(outfile, "%s\n", "help");
  }
  if (args_info->version_given) {
    fprintf(outfile, "%s\n", "version");
  }
  if (args_info->focalLengthX_given) {
    if (args_info->focalLengthX_orig) {
      fprintf(outfile, "%s=\"%s\"\n", "focalLengthX", args_info->focalLengthX_orig);
    } else {
      fprintf(outfile, "%s\n", "focalLengthX");
    }
  }
  if (args_info->focalLengthY_given) {
    if (args_info->focalLengthY_orig) {
      fprintf(outfile, "%s=\"%s\"\n", "focalLengthY", args_info->focalLengthY_orig);
    } else {
      fprintf(outfile, "%s\n", "focalLengthY");
    }
  }
  if (args_info->opticalCenterX_given) {
    if (args_info->opticalCenterX_orig) {
      fprintf(outfile, "%s=\"%s\"\n", "opticalCenterX", args_info->opticalCenterX_orig);
    } else {
      fprintf(outfile, "%s\n", "opticalCenterX");
    }
  }
  if (args_info->opticalCenterY_given) {
    if (args_info->opticalCenterY_orig) {
      fprintf(outfile, "%s=\"%s\"\n", "opticalCenterY", args_info->opticalCenterY_orig);
    } else {
      fprintf(outfile, "%s\n", "opticalCenterY");
    }
  }
  if (args_info->cameraHeight_given) {
    if (args_info->cameraHeight_orig) {
      fprintf(outfile, "%s=\"%s\"\n", "cameraHeight", args_info->cameraHeight_orig);
    } else {
      fprintf(outfile, "%s\n", "cameraHeight");
    }
  }
  if (args_info->pitch_given) {
    if (args_info->pitch_orig) {
      fprintf(outfile, "%s=\"%s\"\n", "pitch", args_info->pitch_orig);
    } else {
      fprintf(outfile, "%s\n", "pitch");
    }
  }
  if (args_info->yaw_given) {
    if (args_info->yaw_orig) {
      fprintf(outfile, "%s=\"%s\"\n", "yaw", args_info->yaw_orig);
    } else {
      fprintf(outfile, "%s\n", "yaw");
    }
  }
  if (args_info->imageWidth_given) {
    if (args_info->imageWidth_orig) {
      fprintf(outfile, "%s=\"%s\"\n", "imageWidth", args_info->imageWidth_orig);
    } else {
      fprintf(outfile, "%s\n", "imageWidth");
    }
  }
  if (args_info->imageHeight_given) {
    if (args_info->imageHeight_orig) {
      fprintf(outfile, "%s=\"%s\"\n", "imageHeight", args_info->imageHeight_orig);
    } else {
      fprintf(outfile, "%s\n", "imageHeight");
    }
  }

  fclose (outfile);

  i = EXIT_SUCCESS;
  return i;
}

void
cameraInfoParser_free (struct CameraInfoParserInfo *args_info)
{
  cameraInfoParser_release (args_info);
}


/* gengetopt_strdup() */
/* strdup.c replacement of strdup, which is not standard */
char *
gengetopt_strdup (const char *s)
{
  char *result = NULL;
  if (!s)
    return result;

  result = (char*)malloc(strlen(s) + 1);
  if (result == (char*)0)
    return (char*)0;
  strcpy(result, s);
  return result;
}

int
cameraInfoParser (int argc, char * const *argv, struct CameraInfoParserInfo *args_info)
{
  return cameraInfoParser2 (argc, argv, args_info, 0, 1, 1);
}

int
cameraInfoParser2 (int argc, char * const *argv, struct CameraInfoParserInfo *args_info, int override, int initialize, int check_required)
{
  int result;

  result = cameraInfoParser_internal (argc, argv, args_info, override, initialize, check_required, NULL);

  if (result == EXIT_FAILURE)
    {
      cameraInfoParser_free (args_info);
      exit (EXIT_FAILURE);
    }

  return result;
}

int
cameraInfoParser_required (struct CameraInfoParserInfo *args_info, const char *prog_name)
{
  int result = EXIT_SUCCESS;

  if (cameraInfoParser_required2(args_info, prog_name, NULL) > 0)
    result = EXIT_FAILURE;

  if (result == EXIT_FAILURE)
    {
      cameraInfoParser_free (args_info);
      exit (EXIT_FAILURE);
    }

  return result;
}

int
cameraInfoParser_required2 (struct CameraInfoParserInfo *args_info, const char *prog_name, const char *additional_error)
{
  int error = 0;

  /* checks for required options */
  if (! args_info->focalLengthX_given)
    {
      fprintf (stderr, "%s: '--focalLengthX' option required%s\n", prog_name, (additional_error ? additional_error : ""));
      error = 1;
    }

  if (! args_info->focalLengthY_given)
    {
      fprintf (stderr, "%s: '--focalLengthY' option required%s\n", prog_name, (additional_error ? additional_error : ""));
      error = 1;
    }

  if (! args_info->opticalCenterX_given)
    {
      fprintf (stderr, "%s: '--opticalCenterX' option required%s\n", prog_name, (additional_error ? additional_error : ""));
      error = 1;
    }

  if (! args_info->opticalCenterY_given)
    {
      fprintf (stderr, "%s: '--opticalCenterY' option required%s\n", prog_name, (additional_error ? additional_error : ""));
      error = 1;
    }

  if (! args_info->cameraHeight_given)
    {
      fprintf (stderr, "%s: '--cameraHeight' option required%s\n", prog_name, (additional_error ? additional_error : ""));
      error = 1;
    }

  if (! args_info->pitch_given)
    {
      fprintf (stderr, "%s: '--pitch' option required%s\n", prog_name, (additional_error ? additional_error : ""));
      error = 1;
    }

  if (! args_info->yaw_given)
    {
      fprintf (stderr, "%s: '--yaw' option required%s\n", prog_name, (additional_error ? additional_error : ""));
      error = 1;
    }

  if (! args_info->imageWidth_given)
    {
      fprintf (stderr, "%s: '--imageWidth' option required%s\n", prog_name, (additional_error ? additional_error : ""));
      error = 1;
    }

  if (! args_info->imageHeight_given)
    {
      fprintf (stderr, "%s: '--imageHeight' option required%s\n", prog_name, (additional_error ? additional_error : ""));
      error = 1;
    }


  /* checks for dependences among options */

  return error;
}

int
cameraInfoParser_internal (int argc, char * const *argv, struct CameraInfoParserInfo *args_info, int override, int initialize, int check_required, const char *additional_error)
{
  int c;	/* Character of the parsed option.  */

  int error = 0;
  struct CameraInfoParserInfo local_args_info;

  if (initialize)
    cameraInfoParser_init (args_info);

  cameraInfoParser_init (&local_args_info);

  optarg = 0;
  optind = 0;
  opterr = 1;
  optopt = '?';

  while (1)
    {
      int option_index = 0;
      char *stop_char;

      static struct option long_options[] = {
        { "help",	0, NULL, 'h' },
        { "version",	0, NULL, 'V' },
        { "focalLengthX",	1, NULL, 0 },
        { "focalLengthY",	1, NULL, 0 },
        { "opticalCenterX",	1, NULL, 0 },
        { "opticalCenterY",	1, NULL, 0 },
        { "cameraHeight",	1, NULL, 0 },
        { "pitch",	1, NULL, 0 },
        { "yaw",	1, NULL, 0 },
        { "imageWidth",	1, NULL, 0 },
        { "imageHeight",	1, NULL, 0 },
        { NULL,	0, NULL, 0 }
      };

      stop_char = 0;
      c = getopt_long (argc, argv, "hV", long_options, &option_index);

      if (c == -1) break;	/* Exit from `while (1)' loop.  */

      switch (c)
        {
        case 'h':	/* Print help and exit.  */
          cameraInfoParser_print_help ();
          cameraInfoParser_free (&local_args_info);
          exit (EXIT_SUCCESS);

        case 'V':	/* Print version and exit.  */
          cameraInfoParser_print_version ();
          cameraInfoParser_free (&local_args_info);
          exit (EXIT_SUCCESS);


        case 0:	/* Long option with no short option */
          /* Focal lenght in horizontal direction in pixels.  */
          if (strcmp (long_options[option_index].name, "focalLengthX") == 0)
          {
            if (local_args_info.focalLengthX_given)
              {
                fprintf (stderr, "%s: `--focalLengthX' option given more than once%s\n", argv[0], (additional_error ? additional_error : ""));
                goto failure;
              }
            if (args_info->focalLengthX_given && ! override)
              continue;
            local_args_info.focalLengthX_given = 1;
            args_info->focalLengthX_given = 1;
            args_info->focalLengthX_arg = strtod (optarg, &stop_char);
            if (!(stop_char && *stop_char == '\0')) {
              fprintf(stderr, "%s: invalid numeric value: %s\n", argv[0], optarg);
              goto failure;
            }
            if (args_info->focalLengthX_orig)
              free (args_info->focalLengthX_orig); /* free previous string */
            args_info->focalLengthX_orig = gengetopt_strdup (optarg);
          }
          /* Focal lenght in vertical direction in pixels.  */
          else if (strcmp (long_options[option_index].name, "focalLengthY") == 0)
          {
            if (local_args_info.focalLengthY_given)
              {
                fprintf (stderr, "%s: `--focalLengthY' option given more than once%s\n", argv[0], (additional_error ? additional_error : ""));
                goto failure;
              }
            if (args_info->focalLengthY_given && ! override)
              continue;
            local_args_info.focalLengthY_given = 1;
            args_info->focalLengthY_given = 1;
            args_info->focalLengthY_arg = strtod (optarg, &stop_char);
            if (!(stop_char && *stop_char == '\0')) {
              fprintf(stderr, "%s: invalid numeric value: %s\n", argv[0], optarg);
              goto failure;
            }
            if (args_info->focalLengthY_orig)
              free (args_info->focalLengthY_orig); /* free previous string */
            args_info->focalLengthY_orig = gengetopt_strdup (optarg);
          }
          /* X-coordinate of optical center in pixels.  */
          else if (strcmp (long_options[option_index].name, "opticalCenterX") == 0)
          {
            if (local_args_info.opticalCenterX_given)
              {
                fprintf (stderr, "%s: `--opticalCenterX' option given more than once%s\n", argv[0], (additional_error ? additional_error : ""));
                goto failure;
              }
            if (args_info->opticalCenterX_given && ! override)
              continue;
            local_args_info.opticalCenterX_given = 1;
            args_info->opticalCenterX_given = 1;
            args_info->opticalCenterX_arg = strtod (optarg, &stop_char);
            if (!(stop_char && *stop_char == '\0')) {
              fprintf(stderr, "%s: invalid numeric value: %s\n", argv[0], optarg);
              goto failure;
            }
            if (args_info->opticalCenterX_orig)
              free (args_info->opticalCenterX_orig); /* free previous string */
            args_info->opticalCenterX_orig = gengetopt_strdup (optarg);
          }
          /* Y-coordinate of optical center in pixels.  */
          else if (strcmp (long_options[option_index].name, "opticalCenterY") == 0)
          {
            if (local_args_info.opticalCenterY_given)
              {
                fprintf (stderr, "%s: `--opticalCenterY' option given more than once%s\n", argv[0], (additional_error ? additional_error : ""));
                goto failure;
              }
            if (args_info->opticalCenterY_given && ! override)
              continue;
            local_args_info.opticalCenterY_given = 1;
            args_info->opticalCenterY_given = 1;
            args_info->opticalCenterY_arg = strtod (optarg, &stop_char);
            if (!(stop_char && *stop_char == '\0')) {
              fprintf(stderr, "%s: invalid numeric value: %s\n", argv[0], optarg);
              goto failure;
            }
            if (args_info->opticalCenterY_orig)
              free (args_info->opticalCenterY_orig); /* free previous string */
            args_info->opticalCenterY_orig = gengetopt_strdup (optarg);
          }
          /* Height of camera above ground in mm.  */
          else if (strcmp (long_options[option_index].name, "cameraHeight") == 0)
          {
            if (local_args_info.cameraHeight_given)
              {
                fprintf (stderr, "%s: `--cameraHeight' option given more than once%s\n", argv[0], (additional_error ? additional_error : ""));
                goto failure;
              }
            if (args_info->cameraHeight_given && ! override)
              continue;
            local_args_info.cameraHeight_given = 1;
            args_info->cameraHeight_given = 1;
            args_info->cameraHeight_arg = strtod (optarg, &stop_char);
            if (!(stop_char && *stop_char == '\0')) {
              fprintf(stderr, "%s: invalid numeric value: %s\n", argv[0], optarg);
              goto failure;
            }
            if (args_info->cameraHeight_orig)
              free (args_info->cameraHeight_orig); /* free previous string */
            args_info->cameraHeight_orig = gengetopt_strdup (optarg);
          }
          /* pitch of camera in degrees (+ve downwards).  */
          else if (strcmp (long_options[option_index].name, "pitch") == 0)
          {
            if (local_args_info.pitch_given)
              {
                fprintf (stderr, "%s: `--pitch' option given more than once%s\n", argv[0], (additional_error ? additional_error : ""));
                goto failure;
              }
            if (args_info->pitch_given && ! override)
              continue;
            local_args_info.pitch_given = 1;
            args_info->pitch_given = 1;
            args_info->pitch_arg = strtod (optarg, &stop_char);
            if (!(stop_char && *stop_char == '\0')) {
              fprintf(stderr, "%s: invalid numeric value: %s\n", argv[0], optarg);
              goto failure;
            }
            if (args_info->pitch_orig)
              free (args_info->pitch_orig); /* free previous string */
            args_info->pitch_orig = gengetopt_strdup (optarg);
          }
          /* yaw of camera in degrees (+ve clockwise).  */
          else if (strcmp (long_options[option_index].name, "yaw") == 0)
          {
            if (local_args_info.yaw_given)
              {
                fprintf (stderr, "%s: `--yaw' option given more than once%s\n", argv[0], (additional_error ? additional_error : ""));
                goto failure;
              }
            if (args_info->yaw_given && ! override)
              continue;
            local_args_info.yaw_given = 1;
            args_info->yaw_given = 1;
            args_info->yaw_arg = strtod (optarg, &stop_char);
            if (!(stop_char && *stop_char == '\0')) {
              fprintf(stderr, "%s: invalid numeric value: %s\n", argv[0], optarg);
              goto failure;
            }
            if (args_info->yaw_orig)
              free (args_info->yaw_orig); /* free previous string */
            args_info->yaw_orig = gengetopt_strdup (optarg);
          }
          /* width of image in pixels.  */
          else if (strcmp (long_options[option_index].name, "imageWidth") == 0)
          {
            if (local_args_info.imageWidth_given)
              {
                fprintf (stderr, "%s: `--imageWidth' option given more than once%s\n", argv[0], (additional_error ? additional_error : ""));
                goto failure;
              }
            if (args_info->imageWidth_given && ! override)
              continue;
            local_args_info.imageWidth_given = 1;
            args_info->imageWidth_given = 1;
            args_info->imageWidth_arg = strtod (optarg, &stop_char);
            if (!(stop_char && *stop_char == '\0')) {
              fprintf(stderr, "%s: invalid numeric value: %s\n", argv[0], optarg);
              goto failure;
            }
            if (args_info->imageWidth_orig)
              free (args_info->imageWidth_orig); /* free previous string */
            args_info->imageWidth_orig = gengetopt_strdup (optarg);
          }
          /* height of image in pixels.  */
          else if (strcmp (long_options[option_index].name, "imageHeight") == 0)
          {
            if (local_args_info.imageHeight_given)
              {
                fprintf (stderr, "%s: `--imageHeight' option given more than once%s\n", argv[0], (additional_error ? additional_error : ""));
                goto failure;
              }
            if (args_info->imageHeight_given && ! override)
              continue;
            local_args_info.imageHeight_given = 1;
            args_info->imageHeight_given = 1;
            args_info->imageHeight_arg = strtod (optarg, &stop_char);
            if (!(stop_char && *stop_char == '\0')) {
              fprintf(stderr, "%s: invalid numeric value: %s\n", argv[0], optarg);
              goto failure;
            }
            if (args_info->imageHeight_orig)
              free (args_info->imageHeight_orig); /* free previous string */
            args_info->imageHeight_orig = gengetopt_strdup (optarg);
          }

          break;
        case '?':	/* Invalid option.  */
          /* `getopt_long' already printed an error message.  */
          goto failure;

        default:	/* bug: option not considered.  */
          fprintf (stderr, "%s: option unknown: %c%s\n", CAMERAINFOPARSER_PACKAGE, c, (additional_error ? additional_error : ""));
          abort ();
        } /* switch */
    } /* while */



  if (check_required)
    {
      error += cameraInfoParser_required2 (args_info, argv[0], additional_error);
    }

  cameraInfoParser_release (&local_args_info);

  if ( error )
    return (EXIT_FAILURE);

  return 0;

failure:

  cameraInfoParser_release (&local_args_info);
  return (EXIT_FAILURE);
}

#ifndef CONFIG_FILE_LINE_SIZE
#define CONFIG_FILE_LINE_SIZE 2048
#endif
#define ADDITIONAL_ERROR " in configuration file "

#define CONFIG_FILE_LINE_BUFFER_SIZE (CONFIG_FILE_LINE_SIZE+3)
/* 3 is for "--" and "=" */

char my_argv[CONFIG_FILE_LINE_BUFFER_SIZE+1];

int cameraInfoParser_configfile (char * const filename, struct CameraInfoParserInfo *args_info, int override, int initialize, int check_required)
{
  FILE* file;
  char linebuf[CONFIG_FILE_LINE_SIZE];
  int line_num = 0;
  int i, result, equal;
  char *fopt, *farg;
  char *str_index;
  size_t len, next_token;
  char delimiter;
  int my_argc = 0;
  char **my_argv_arg;
  char *additional_error;

  /* store the program name */
  cmd_line_list_tmp = (struct line_list *) malloc (sizeof (struct line_list));
  cmd_line_list_tmp->next = cmd_line_list;
  cmd_line_list = cmd_line_list_tmp;
  cmd_line_list->string_arg = gengetopt_strdup (CAMERAINFOPARSER_PACKAGE);

  if ((file = fopen(filename, "r")) == NULL)
    {
      fprintf (stderr, "%s: Error opening configuration file '%s'\n",
               CAMERAINFOPARSER_PACKAGE, filename);
      result = EXIT_FAILURE;
      goto conf_failure;
    }

  while ((fgets(linebuf, CONFIG_FILE_LINE_SIZE, file)) != NULL)
    {
      ++line_num;
      my_argv[0] = '\0';
      len = strlen(linebuf);
      if (len > (CONFIG_FILE_LINE_BUFFER_SIZE-1))
        {
          fprintf (stderr, "%s:%s:%d: Line too long in configuration file\n",
                   CAMERAINFOPARSER_PACKAGE, filename, line_num);
          result = EXIT_FAILURE;
          goto conf_failure;
        }

      /* find first non-whitespace character in the line */
      next_token = strspn ( linebuf, " \t\r\n");
      str_index  = linebuf + next_token;

      if ( str_index[0] == '\0' || str_index[0] == '#')
        continue; /* empty line or comment line is skipped */

      fopt = str_index;

      /* truncate fopt at the end of the first non-valid character */
      next_token = strcspn (fopt, " \t\r\n=");

      if (fopt[next_token] == '\0') /* the line is over */
        {
          farg  = NULL;
          equal = 0;
          goto noarg;
        }

      /* remember if equal sign is present */
      equal = (fopt[next_token] == '=');
      fopt[next_token++] = '\0';

      /* advance pointers to the next token after the end of fopt */
      next_token += strspn (fopt + next_token, " \t\r\n");
      /* check for the presence of equal sign, and if so, skip it */
      if ( !equal )
        if ((equal = (fopt[next_token] == '=')))
          {
            next_token++;
            next_token += strspn (fopt + next_token, " \t\r\n");
          }
      str_index  += next_token;

      /* find argument */
      farg = str_index;
      if ( farg[0] == '\"' || farg[0] == '\'' )
        { /* quoted argument */
          str_index = strchr (++farg, str_index[0] ); /* skip opening quote */
          if (! str_index)
            {
              fprintf
                (stderr,
                 "%s:%s:%d: unterminated string in configuration file\n",
                 CAMERAINFOPARSER_PACKAGE, filename, line_num);
              result = EXIT_FAILURE;
              goto conf_failure;
            }
        }
      else
        { /* read up the remaining part up to a delimiter */
          next_token = strcspn (farg, " \t\r\n#\'\"");
          str_index += next_token;
        }

      /* truncate farg at the delimiter and store it for further check */
      delimiter = *str_index, *str_index++ = '\0';

      /* everything but comment is illegal at the end of line */
      if (delimiter != '\0' && delimiter != '#')
        {
          str_index += strspn(str_index, " \t\r\n");
          if (*str_index != '\0' && *str_index != '#')
            {
              fprintf
                (stderr,
                 "%s:%s:%d: malformed string in configuration file\n",
                 CAMERAINFOPARSER_PACKAGE, filename, line_num);
              result = EXIT_FAILURE;
              goto conf_failure;
            }
        }

    noarg:
      ++my_argc;
      len = strlen(fopt);

      strcat (my_argv, len > 1 ? "--" : "-");
      strcat (my_argv, fopt);
      if (len > 1 && ((farg &&*farg) || equal))
          strcat (my_argv, "=");
      if (farg && *farg)
          strcat (my_argv, farg);

      cmd_line_list_tmp = (struct line_list *) malloc (sizeof (struct line_list));
      cmd_line_list_tmp->next = cmd_line_list;
      cmd_line_list = cmd_line_list_tmp;
      cmd_line_list->string_arg = gengetopt_strdup(my_argv);
    } /* while */

  ++my_argc; /* for program name */
  my_argv_arg = (char **) malloc((my_argc+1) * sizeof(char *));
  cmd_line_list_tmp = cmd_line_list;
  for (i = my_argc - 1; i >= 0; --i) {
    my_argv_arg[i] = cmd_line_list_tmp->string_arg;
    cmd_line_list_tmp = cmd_line_list_tmp->next;
  }
  my_argv_arg[my_argc] = 0;

  additional_error = (char *)malloc(strlen(filename) + strlen(ADDITIONAL_ERROR) + 1);
  strcpy (additional_error, ADDITIONAL_ERROR);
  strcat (additional_error, filename);
  result =
    cameraInfoParser_internal (my_argc, my_argv_arg, args_info, override, initialize, check_required, additional_error);

  free (additional_error);
  free (my_argv_arg);

conf_failure:
  if (file)
    fclose(file);

  free_cmd_list();
  if (result == EXIT_FAILURE)
    {
      cameraInfoParser_free (args_info);
      exit (EXIT_FAILURE);
    }

  return result;
}

#ifdef __cplusplus
}
#endif
