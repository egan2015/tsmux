## Process this file with automake to produce Makefile.in

pkgincludedir = $(includedir)/dvbpsi

lib_LTLIBRARIES = libdvbpsi.la

libdvbpsi_la_SOURCES = dvbpsi.c dvbpsi_private.h \
                       psi.c \
                       demux.c \
                       descriptor.c \
                       $(tables_src) \
                       $(descriptors_src)

libdvbpsi_la_LDFLAGS = -version-info 11:0:0 -no-undefined

pkginclude_HEADERS = dvbpsi.h psi.h descriptor.h demux.h \
                     tables/pat.h tables/pmt.h tables/sdt.h tables/eit.h \
                     tables/cat.h tables/nit.h tables/tot.h tables/sis.h \
		     tables/bat.h tables/rst.h \
		     tables/atsc_vct.h tables/atsc_stt.h \
		     tables/atsc_eit.h tables/atsc_mgt.h \
		     tables/atsc_ett.h \
		     descriptors/dr.h

mpegdrincludedir = $(pkgincludedir)/mpeg

mpegdrinclude_HEADERS = descriptors/mpeg/dr_02.h \
                     descriptors/mpeg/dr_03.h \
                     descriptors/mpeg/dr_04.h \
                     descriptors/mpeg/dr_05.h \
                     descriptors/mpeg/dr_06.h \
                     descriptors/mpeg/dr_07.h \
                     descriptors/mpeg/dr_08.h \
                     descriptors/mpeg/dr_09.h \
                     descriptors/mpeg/dr_0a.h \
                     descriptors/mpeg/dr_0b.h \
                     descriptors/mpeg/dr_0c.h \
                     descriptors/mpeg/dr_0d.h \
                     descriptors/mpeg/dr_0e.h \
                     descriptors/mpeg/dr_0f.h \
                     descriptors/mpeg/dr_10.h \
                     descriptors/mpeg/dr_11.h \
                     descriptors/mpeg/dr_12.h \
                     descriptors/mpeg/dr_13.h \
                     descriptors/mpeg/dr_14.h \
                     descriptors/mpeg/dr_1b.h \
                     descriptors/mpeg/dr_1c.h \
                     descriptors/mpeg/dr_24.h

dvbdrincludedir = $(pkgincludedir)/dvb

dvbdrinclude_HEADERS = descriptors/dvb/dr_40.h \
                     descriptors/dvb/dr_41.h \
                     descriptors/dvb/dr_42.h \
                     descriptors/dvb/dr_43.h \
                     descriptors/dvb/dr_44.h \
                     descriptors/dvb/dr_45.h \
                     descriptors/dvb/dr_47.h \
                     descriptors/dvb/dr_48.h \
                     descriptors/dvb/dr_49.h \
                     descriptors/dvb/dr_4a.h \
                     descriptors/dvb/dr_4b.h \
                     descriptors/dvb/dr_4c.h \
                     descriptors/dvb/dr_4d.h \
                     descriptors/dvb/dr_4e.h \
                     descriptors/dvb/dr_4f.h \
                     descriptors/dvb/dr_50.h \
                     descriptors/dvb/dr_52.h \
                     descriptors/dvb/dr_53.h \
                     descriptors/dvb/dr_54.h \
                     descriptors/dvb/dr_55.h \
                     descriptors/dvb/dr_56.h \
                     descriptors/dvb/dr_58.h \
                     descriptors/dvb/dr_59.h \
                     descriptors/dvb/dr_5a.h \
                     descriptors/dvb/dr_62.h \
                     descriptors/dvb/dr_66.h \
                     descriptors/dvb/dr_69.h \
                     descriptors/dvb/dr_73.h \
                     descriptors/dvb/dr_76.h \
                     descriptors/dvb/dr_7c.h

atscdrincludedir = $(pkgincludedir)/atsc

atscdrinclude_HEADERS = descriptors/atsc/dr_81.h \
                     descriptors/atsc/dr_86.h \
                     descriptors/atsc/dr_a0.h \
                     descriptors/atsc/dr_a1.h

customdrincludedir = $(pkgincludedir)/custom

customdrinclude_HEADERS = descriptors/custom/dr_83_eacem.h \
                     descriptors/custom/dr_8a_scte.h

typesincludedir = $(pkgincludedir)/types

typesinclude_HEADERS = descriptors/types/aac_profile.h

descriptors_src = descriptors/mpeg/dr_02.c \
                  descriptors/mpeg/dr_03.c \
                  descriptors/mpeg/dr_04.c \
                  descriptors/mpeg/dr_05.c \
                  descriptors/mpeg/dr_06.c \
                  descriptors/mpeg/dr_07.c \
                  descriptors/mpeg/dr_08.c \
                  descriptors/mpeg/dr_09.c \
                  descriptors/mpeg/dr_0a.c \
                  descriptors/mpeg/dr_0b.c \
                  descriptors/mpeg/dr_0c.c \
                  descriptors/mpeg/dr_0d.c \
                  descriptors/mpeg/dr_0e.c \
                  descriptors/mpeg/dr_0f.c \
                  descriptors/mpeg/dr_10.c \
                  descriptors/mpeg/dr_11.c \
                  descriptors/mpeg/dr_12.c \
                  descriptors/mpeg/dr_13.c \
                  descriptors/mpeg/dr_14.c \
                  descriptors/mpeg/dr_1b.c \
                  descriptors/mpeg/dr_1c.c \
                  descriptors/mpeg/dr_24.c \
		  descriptors/dvb/dr_40.c \
		  descriptors/dvb/dr_41.c \
                  descriptors/dvb/dr_42.c \
                  descriptors/dvb/dr_43.c \
                  descriptors/dvb/dr_44.c \
                  descriptors/dvb/dr_45.c \
                  descriptors/dvb/dr_47.c \
                  descriptors/dvb/dr_48.c \
                  descriptors/dvb/dr_49.c \
                  descriptors/dvb/dr_4a.c \
                  descriptors/dvb/dr_4b.c \
                  descriptors/dvb/dr_4c.c \
                  descriptors/dvb/dr_4d.c \
                  descriptors/dvb/dr_4e.c \
                  descriptors/dvb/dr_4f.c \
                  descriptors/dvb/dr_50.c \
                  descriptors/dvb/dr_52.c \
                  descriptors/dvb/dr_53.c \
                  descriptors/dvb/dr_54.c \
                  descriptors/dvb/dr_55.c \
                  descriptors/dvb/dr_56.c \
                  descriptors/dvb/dr_58.c \
                  descriptors/dvb/dr_59.c \
                  descriptors/dvb/dr_5a.c \
                  descriptors/dvb/dr_62.c \
                  descriptors/dvb/dr_66.c \
                  descriptors/dvb/dr_69.c \
                  descriptors/dvb/dr_73.c \
                  descriptors/dvb/dr_76.c \
                  descriptors/dvb/dr_7c.c \
                  descriptors/atsc/dr_81.c \
                  descriptors/atsc/dr_86.c \
		  descriptors/atsc/dr_a0.c \
		  descriptors/atsc/dr_a1.c \
		  descriptors/custom/dr_83_eacem.c \
		  descriptors/custom/dr_8a_scte.c

tables_src = tables/pat.c tables/pat_private.h \
             tables/pmt.c tables/pmt_private.h \
             tables/sdt.c tables/sdt_private.h \
             tables/eit.c tables/eit_private.h \
             tables/cat.c tables/cat_private.h \
             tables/nit.c tables/nit_private.h \
             tables/tot.c tables/tot_private.h \
             tables/sis.c tables/sis_private.h \
	     tables/bat.c tables/bat_private.h \
	     tables/rst.c tables/rst_private.h \
	     tables/atsc_vct.c tables/atsc_vct.h \
	     tables/atsc_stt.c tables/atsc_stt.h \
	     tables/atsc_eit.c tables/atsc_eit.h \
	     tables/atsc_ett.c tables/atsc_ett.h \
	     tables/atsc_mgt.c tables/atsc_mgt.h
