## $Header: /var/cvs/mbdyn/mbdyn/mbdyn-1.0/modules/module-aerodyn/Makefile.inc,v 1.5 2009/06/12 11:14:56 masarati Exp $

MODULE_LINK=-L. -lkitefastlib -L. -lnwtclibs -L. -lmoordynlib -L. -lkiteaerodynlib -L. -lvsmlib -L. -lactuatordisklib -L. -lairfoilinfolib -L. -lifwlib # -Wl,-no_compact_unwind

# for future reference:
# -Wl,-no_compact_unwind
