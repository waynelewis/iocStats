
#include <dbStaticLib.h>
#include <dbAccess.h>

#include "devIocStats.h"

struct link *devIocStatsGetDevLink(struct dbCommon *prec)
{
    struct link *ret = NULL;
    dbFldDes *pfdes = NULL;
    DBENTRY ent;

    dbInitEntry(pdbbase, &ent);

    if(dbFindRecord(&ent, prec->name))
        goto done;

    if(!dbFindField(&ent, "INP")) {
        pfdes = ent.pflddes;
        goto done;
    }

    if(!dbFindField(&ent, "OUT")) {
        pfdes = ent.pflddes;
        goto done;
    }

done:
    dbFinishEntry(&ent);

    if(pfdes) {
        switch(pfdes->field_type) {
        case DBF_INLINK:
        case DBF_OUTLINK:
            ret = (struct link*)(pfdes->offset+(char*)prec);
            break;
        default:
            break;
        }
    }

    return ret;
}
