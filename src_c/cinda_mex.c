/* CINDA - (CIrculation Network based Data-Association)
* 
* This is an implementation of the CINDA framework. It solves the unit-capacity min-cost
* circulation problem in multi-object tracking problem[1].
* 
* Quite a few functions here come from cs2[2], whose copyright belongs to IG Systems.
* Thus for any use except for pure academic research and evaluation purposes requires 
* a licence (igsys@eclipse.com).
*
* Here we also want to thank Hamed Pirsiavash for his initial matlab interface 
* implementation.
*
* [1] Congchao Wang, Yizhi Wang, Guoqiang Yu, "Efficient Global Multi-object Tracking 
*     Under Minimum-cost Circulation Framework", arXiv:1911.00796.
* [2] Andrew V. Goldberg, "An efficient implementation of a scaling minimum-cost flow 
*     algorithm", Journal of algorithms, vol. 22, no. 1, pp. 1–29, 1997.
*
* Contact: Congchao Wang (ccwang AT vt DOT edu)
* Affiliation: Yu-Lab, CBIL, Virginia Tech
* Date: September 2020
*/

#include <mex.h>
#include "cinda_funcs.c"

void mex_print(mxArray *plhs[], node *ndp, arc *arp, long nmin, double *cost)
{
  node *i, *ii;
  arc *a, *b;
  long ni;
  price_t cost2;

  plhs[0] = mxCreateDoubleScalar((double)*cost);
  long j = 0;
  long f = 0;
  /*Compute number of edges with nonzero flow*/
  for (i = ndp; i < ndp + n; i++)
  {
    ni = N_NODE(i);
    for (a = i->suspended; a != (i + 1)->suspended; a++)
      if (cap[N_ARC(a)] > 0 && cap[N_ARC(a)] - (a->r_cap) > 0)
        j++;
  }
  
  plhs[1] = mxCreateDoubleMatrix(j, 1, mxREAL);
  plhs[2] = mxCreateDoubleMatrix(j, 1, mxREAL);
  plhs[3] = mxCreateDoubleMatrix(j, 1, mxREAL);
  double *ftail = (double *)mxGetPr(plhs[1]);
  double *fhead = (double *)mxGetPr(plhs[2]);
  double *flow = (double *)mxGetPr(plhs[3]);

  j = 0;
  for (i = ndp; i < ndp + n; i++)
  {
    ni = N_NODE(i);
    for (a = i->suspended; a != (i + 1)->suspended; a++)
    {
      if (cap[N_ARC(a)] > 0)
      {
        f = cap[N_ARC(a)] - (a->r_cap);
        if (f > 0)
        {
          ftail[j] = ni;
          fhead[j] = N_NODE(a->head);
          flow[j] = f;
          j++;
        }
      }
    }
  }
  
  /*save trajectories*/
  plhs[4] = mxCreateDoubleMatrix(j, 1, mxREAL);
  double *tracks = (double *)mxGetPr(plhs[4]);

  j = 0;
  price_t cost3 = 0;
  i = ndp; // start point and end point
  if (n % 2 == 0)
    ni = N_NODE(ndp + n - 1); //min-cost flow formulation
  else
    ni = N_NODE(ndp); //min-cost circulation formulation
  
  /* One arc will never appear in two circles or paths. However, 
  * a node may be shared, and thus we need to check if an arc 
  * from current node is already visited when tranverse other circles. 
  * 
  * This condition will not be encountered if neither nodes and arcs
  * can be shared in different circles. For example, the regular graph
  * structure in multi-object tracking problem.
  */
  bool *arc_visited = (bool *)calloc(2 * m, sizeof(bool));
  memset(&arc_visited[0], false, 2 * m * sizeof(bool));

  for (a = i->suspended; a != (i + 1)->suspended; a++)
  {
    if (cap[N_ARC(a)] > 0 && cap[N_ARC(a)] - (a->r_cap) > 0)
    {
      arc_visited[N_ARC(a)] = true;

      //forward track the path
      b = a;
      while (N_NODE(b->head) != ni)
      {
        cost3 += b->cost;
        tracks[j] = N_NODE(b->head);
        j++;
        ii = b->head;
        for (b = ii->suspended; b != (ii + 1)->suspended; b++)
        {
          if (cap[N_ARC(b)] > 0 && cap[N_ARC(b)] - (b->r_cap) > 0 && !arc_visited[N_ARC(b)]){
            arc_visited[N_ARC(b)] = true;
            break;
          }
        }
        /*if (N_NODE(b->head)==N_NODE(ii))
          mexErrMsgTxt("Repeated loop");*/
      }

      tracks[j] = cost3 + b->cost;
      cost3 = 0;
      j++;
    }
  }
  //printf("we should have %ld circles\n", j);
#ifdef COMP_DUALS
  /* find minimum price */
  cost2 = MAX_32;
  FOR_ALL_NODES_i
  {
    cost2 = MIN(*cost, i->price);
  }
  FOR_ALL_NODES_i
  {
    printf("p %7ld %7.2lld\n", N_NODE(i), i->price - cost2);
  }
#endif

  free(arc_visited);
  /*printf("c\n");*/
}

/*-----------------------------------------------------------
  mexparse (num_nodes,num_arcs,) :                                                   
       1. Reads minimum-cost flow problem in  DIMACS format.   
       2. Prepares internal data representation.

   types: 'arc' and 'node' must be predefined

   type   node   must contain the field 'first': 

   typedef
     struct node_st
       {
          arc_st        *first;    ..  first outgoing arc 
          ....................
       }
    node;
--------------------------------------------------------------*/
int mexparse(const mxArray *prhs[], long *n_ad, long *m_ad, node **nodes_ad,
             arc **arcs_ad, long *node_min_ad, price_t *m_c_ad,
             long **cap_ad, long *f_sc)

{

#define MAXLINE 100        /* max line length in the input file */
#define ARC_FIELDS 5       /* no of fields in arc line  */
#define NODE_FIELDS 2      /* no of fields in node line  */
#define P_FIELDS 3         /* no of fields in problem line */
#define PROBLEM_TYPE "min" /* name of problem type*/

  long inf_cap = 0;
  long n,         /* internal number of nodes */
      node_min,   /* minimal no of node  */
      node_max,   /* maximal no of nodes */
      *arc_first, /* internal array for holding
                                     - node degree
                                     - position of the first outgoing arc */
      *arc_tail,  /* internal array: tails of the arcs */
                  /* temporary variables carrying no of nodes */
      head, tail, i;

  long m, /* internal number of arcs */
      /* temporary variables carrying no of arcs */
      last, arc_num, arc_new_num;

  node *nodes, /* pointers to the node structure */
      *head_p,
      *ndp,
      *in,
      *jn;

  arc *arcs, /* pointers to the arc structure */
      *arc_current,
      *arc_new,
      *arc_tmp;

  long excess, /* supply/demand of the node */
      exnode,  /* source/sink node indexes */
      low,     /* lowest flow through the arc */
      acap;    /* capacity */

  price_t cost, /* arc cost */
      m_c;      /* maximum arc cost */

  long *cap; /* array of capacities */

  excess_t total_p, /* total supply */
      total_n,      /* total demand */
      cap_out,      /* sum of outgoing capacities */
      cap_in;       /* sum of incoming capacities */

  long no_lines = 0,   /* no of current input line */
      no_plines = 0,   /* no of problem-lines */
      no_nlines = 0,   /* no of node lines */
      no_alines = 0,   /* no of arc-lines */
      pos_current = 0; /* 2*no_alines */

  char in_line[MAXLINE], /* for reading input line */
      pr_type[3];        /* for reading type of the problem */

  int k,      /* temporary */
      err_no; /* no of detected error */

  /* The main loop:
        -  reads the line of the input,
        -  analises its type,
        -  checks correctness of parameters,
        -  puts data to the arrays,
        -  does service functions
*/

  /*int mexparse(const mxArray *prhs[],long *n_ad, long *m_ad, node **nodes_ad, 
	     arc **arcs_ad, long *node_min_ad, price_t *m_c_ad, 
	     long **cap_ad, long *f_sc)*/
  *f_sc = (long)mxGetScalar(prhs[0]);
  n = (long)mxGetScalar(prhs[1]);
  m = (long)mxGetScalar(prhs[2]);

  /* allocating memory for  'nodes', 'arcs'  and internal arrays */
  nodes = (node *)calloc(n + 2, sizeof(node));
  arcs = (arc *)calloc(2 * m + 1, sizeof(arc));
  cap = (long *)calloc(2 * m, sizeof(long));
  arc_tail = (long *)calloc(2 * m, sizeof(long));
  arc_first = (long *)calloc(n + 2, sizeof(long));

  /* arc_first [ 0 .. n+1 ] = 0 - initialized by calloc */
  for (in = nodes; in <= nodes + n; in++)
    in->excess = 0;
  if (nodes == NULL || arcs == NULL ||
      arc_first == NULL || arc_tail == NULL)
  /* memory is not allocated */
  {
    mexErrMsgTxt("Could not allocate memory");
  }

  /* setting pointer to the first arc */
  arc_current = arcs;
  node_max = 0;
  node_min = n;
  m_c = 0;
  total_p = total_n = 0;

  for (ndp = nodes; ndp < nodes + n; ndp++)
    ndp->excess = 0;

  /*printf("(n,m,scale) = (%d,%d,%d)\n",n,m,*f_sc);*/

  /* Store excesses*/
  long len = MAX(mxGetN(prhs[3]), mxGetM(prhs[3]));
  double *mexnode = (double *)mxGetPr(prhs[3]);
  double *mexcess = (double *)mxGetPr(prhs[4]);
  i = 0;
  for (i = 0; i < len; i++)
  {
    no_nlines++;
    exnode = (long)mexnode[i];
    excess = (long)mexcess[i];
    (nodes + exnode)->excess = excess;
    if (excess > 0)
      total_p += excess;
    if (excess < 0)
      total_n -= excess;
    /*printf("(i,val,np,nn) = (%d,%d,%d,%d)\n",exnode,excess,total_p,total_n);*/
  }
  /* Store arcs*/
  double *mtail = (double *)mxGetPr(prhs[5]);
  double *mhead = (double *)mxGetPr(prhs[6]);
  double *mlow = (double *)mxGetPr(prhs[7]);
  double *macap = (double *)mxGetPr(prhs[8]);
  double *mcost = (double *)mxGetPr(prhs[9]);
  len = MAX(mxGetN(prhs[5]), mxGetM(prhs[5]));

  for (i = 0; i < len; i++)
  {
    no_lines++;
    tail = (long)mtail[i];
    head = (long)mhead[i];
    low = (long)mlow[i];
    acap = (long)macap[i];
    cost = (price_t)mcost[i];
    if (tail < 0 || tail > n ||
        head < 0 || head > n)
    {
      mexErrMsgTxt("Wrong value of nodes");
    }
    if (acap < 0)
    {
      acap = MAX_32;
      if (!inf_cap)
      {
        inf_cap = 1;
        printf("Negative capacity set to infinity");
      }
    }
    if (low < 0 || low > acap)
    {
      mexErrMsgTxt("Lower bound exceeds upper boud");
    }

    /* no of arcs incident to node i is placed in arc_first[i+1] */
    arc_first[tail + 1]++;
    arc_first[head + 1]++;
    in = nodes + tail;
    jn = nodes + head;

    /* storing information about the arc */
    arc_tail[pos_current] = tail;
    arc_tail[pos_current + 1] = head;
    arc_current->head = jn;
    arc_current->r_cap = acap - low;
    cap[pos_current] = acap;
    arc_current->cost = cost;
    arc_current->sister = arc_current + 1;
    (arc_current + 1)->head = nodes + tail;
    (arc_current + 1)->r_cap = 0;
    cap[pos_current + 1] = 0;
    (arc_current + 1)->cost = -cost;
    (arc_current + 1)->sister = arc_current;

    in->excess -= low;
    jn->excess += low;

    /* searching for minimum and maximum node */
    if (head < node_min)
      node_min = head;
    if (tail < node_min)
      node_min = tail;
    if (head > node_max)
      node_max = head;
    if (tail > node_max)
      node_max = tail;

    if (cost < 0)
      cost = -cost;
    if (cost > m_c && acap > 0)
      m_c = cost;

    no_alines++;
    arc_current += 2;
    pos_current += 2;
  } /* end of input loop */

  /********** ordering arcs - linear time algorithm ***********/

  /* first arc from the first node */
  (nodes + node_min)->first = arcs;

  /* before below loop arc_first[i+1] is the number of arcs outgoing from i;
   after this loop arc_first[i] is the position of the first 
   outgoing from node i arcs after they would be ordered;
   this value is transformed to pointer and written to node.first[i]
   */

  for (i = node_min + 1; i <= node_max + 1; i++)
  {
    arc_first[i] += arc_first[i - 1];
    (nodes + i)->first = arcs + arc_first[i];
  }

  for (i = node_min; i < node_max; i++) /* scanning all the nodes  
                                            exept the last*/
  {

    last = ((nodes + i + 1)->first) - arcs;
    /* arcs outgoing from i must be cited    
                              from position arc_first[i] to the position
                              equal to initial value of arc_first[i+1]-1  */

    for (arc_num = arc_first[i]; arc_num < last; arc_num++)
    {
      tail = arc_tail[arc_num];

      while (tail != i)
      /* the arc no  arc_num  is not in place because arc cited here
             must go out from i;
             we'll put it to its place and continue this process
             until an arc in this position would go out from i */

      {
        arc_new_num = arc_first[tail];
        arc_current = arcs + arc_num;
        arc_new = arcs + arc_new_num;

        /* arc_current must be cited in the position arc_new    
	       swapping these arcs:                                 */

        head_p = arc_new->head;
        arc_new->head = arc_current->head;
        arc_current->head = head_p;

        acap = cap[arc_new_num];
        cap[arc_new_num] = cap[arc_num];
        cap[arc_num] = acap;

        acap = arc_new->r_cap;
        arc_new->r_cap = arc_current->r_cap;
        arc_current->r_cap = acap;

        cost = arc_new->cost;
        arc_new->cost = arc_current->cost;
        arc_current->cost = cost;

        if (arc_new != arc_current->sister)
        {
          arc_tmp = arc_new->sister;
          arc_new->sister = arc_current->sister;
          arc_current->sister = arc_tmp;

          (arc_current->sister)->sister = arc_current;
          (arc_new->sister)->sister = arc_new;
        }

        arc_tail[arc_num] = arc_tail[arc_new_num];
        arc_tail[arc_new_num] = tail;

        /* we increase arc_first[tail]  */
        arc_first[tail]++;

        tail = arc_tail[arc_num];
      }
    }
    /* all arcs outgoing from  i  are in place */
  }

  /* -----------------------  arcs are ordered  ------------------------- */

  /*------------ testing network for possible excess overflow ---------*/

  for (ndp = nodes + node_min; ndp <= nodes + node_max; ndp++)
  {
    cap_in = (ndp->excess);
    cap_out = -(ndp->excess);
    for (arc_current = ndp->first; arc_current != (ndp + 1)->first;
         arc_current++)
    {
      arc_num = arc_current - arcs;
      if (cap[arc_num] > 0)
        cap_out += cap[arc_num];
      if (cap[arc_num] == 0)
        cap_in += cap[(arc_current->sister) - arcs];
    }
  }

  if ((node_min < 0) || (node_min > 1)) /* unbalanced problem */
  {
    mexErrMsgTxt("Unbalanced problem");
  }

  /* ----------- assigning output values ------------*/
  *m_ad = m;
  *n_ad = node_max - node_min + 1;
  *node_min_ad = node_min;
  *nodes_ad = nodes + node_min;
  *arcs_ad = arcs;
  *m_c_ad = m_c;
  *cap_ad = cap;

  /* free internal memory */
  free(arc_first);
  free(arc_tail);

  /* Thanks God! All is done! */
  return (0);
}
/* --------------------   end of parser  -------------------*/

/*int main (int argc, char **argv)*/
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

  double t;
  arc *arp;
  node *ndp;
  long n, m, m2, nmin;

  double cost;

  price_t c_max;
  long f_sc;
  long *cap;

  /*f_sc = (long) (( argc > 1 ) ? atoi( argv[1] ): SCALE_DEFAULT);*/

  /*
  printf ("c CS 4.6\n");
  printf ("c Commercial use requires a licence\n");
  printf ("c contact igsys@eclipse.net\nc\n");
  */
  if (nrhs < 10)
    mexErrMsgTxt("Incorrect number of input arguments");
  if (nlhs < 5)
    mexErrMsgTxt("Incorrect number of output arguments");
  
  mexparse(prhs, &n, &m, &ndp, &arp, &nmin, &c_max, &cap, &f_sc);

  nodes = ndp;

#ifdef CHECK_SOLUTION
  node_balance = (long long int *)calloc(n + 1, sizeof(long long int));
  node *i;
  for (i = ndp; i < ndp + n; i++)
  {
    node_balance[i - ndp] = i->excess;
  }
#endif

  m2 = 2 * m;
  //printf ("c nodes: %15ld     arcs:  %15ld\n", n, m );
  
//   t = timer();
  cs2(n, m2, ndp, arp, f_sc, c_max, cap, &cost);
//   t = timer() - t;
  /*
  printf ("c time:  %15.2f     cost:  %15.0f\n", t, cost);  
  printf ("c refines:    %10ld     discharges: %10ld\n", n_refine, n_discharge);
  printf ("c pushes:     %10ld     relabels:   %10ld\n", n_push, n_relabel);
  printf ("c updates:    %10ld     u-scans:    %10ld\n", n_update, n_scan);
  printf ("c p-refines:  %10ld     r-scans:    %10ld\n", n_prefine, n_prscan);
  printf ("c dfs-scans:  %10ld     bad-in:     %4ld  + %2ld\n", n_prscan1, n_bad_pricein, n_bad_relabel);
  */

#ifdef CHECK_SOLUTION
  printf("c checking feasibility...\n");
  if (check_feas(ndp, arp))
    printf("c ...OK\n");
  else
    printf("c ERROR: solution infeasible\n");
  printf("c computing prices and checking CS...\n");
  compute_prices();
  if (check_cs())
    printf("c ...OK\n");
  else
    printf("ERROR: CS violation\n");
#endif

  mex_print(plhs, ndp, arp, nmin, &cost);

  free(cap);
  free(nodes - nmin);
  free(arcs);
  free(buckets);
#ifdef CHECK_SOLUTION
  free(node_balance);
#endif
}
