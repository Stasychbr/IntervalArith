## -*- texinfo -*-
## @documentencoding UTF-8
## @deftypemethod {@@kinterval} {M =} diag (@var{V})
## @deftypemethodx {@@kinterval} {V =} diag (@var{M})
## @deftypemethodx {@@kinterval} {M =} diag (@var{V}, @var{K})
## @deftypemethodx {@@kinterval} {M =} diag (@var{V}, @var{M}, @var{N})
## @deftypemethodx {@@kinterval} {V =} diag (@var{M}, @var{K})
##
## Create a diagonal matrix @var{M} with vector @var{V} on diagonal @var{K} or
## extract a vector @var{V} from the @var{K}-th diagonal of matrix @var{M}.
##
## With three arguments, create a matrix of size @var{M}×@var{N}.
##
## @example
## @group
## diag (kinterval (1 : 3))
##   @result{} ans = 3×3 interval matrix
##
##        [1]   [0]   [0]
##        [0]   [2]   [0]
##        [0]   [0]   [3]
## @end group
## @end example
## @seealso{@@kinterval/tril, @@kinterval/triu}
## @end deftypemethod

function x = diag (x, varargin)

  if (nargin >= 2 && isa (varargin{1}, 'kinterval'))
    error ('diag: invalid second argument; it must not be an interval');
  endif
  if (nargin >= 3 && isa (varargin{2}, 'kinterval'))
    error ('diag: invalid third argument; it must not be an interval');
  endif
  if (nargin > 3)
    print_usage ();
    return
  endif

  l = diag (x.inf, varargin{:});
  u = diag (x.sup, varargin{:});

  l(l == 0) = -0;

  x.inf = l;
  x.sup = u;

endfunction

%!assert (diag (kinterval (-inf, inf)) == "[Entire]");
%!assert (diag (kinterval ()) == "[Empty]");
%!assert (numel (diag (kinterval ([]))), 0);
%!assert (isequal (diag (kinterval (magic (3))), kinterval ([8; 5; 2])));
%!assert (isequal (diag (kinterval ([8 5 3])), kinterval ([8 0 0; 0 5 0; 0 0 3])));
%!assert (isequal (diag (kinterval (1:2), 2, 3), kinterval ([1 0 0; 0 2 0])));
