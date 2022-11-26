https://powcoder.com
代写代考加微信 powcoder
Assignment Project Exam Help
Add WeChat powcoder
https://powcoder.com
代写代考加微信 powcoder
Assignment Project Exam Help
Add WeChat powcoder

% pathsearch.pl

% COMP3411/9414/9814 Artificial Intelligence, UNSW, Alan Blair

% This file provides code for insert_legs(), head_member() and build_path()
% used by bfsdijkstra(), ucsdijkstra(), greedy() and astar().

% insert_legs(Generated, Legs, Generated1).
% insert new legs into list of generated legs,
% by repeatedly calling insert_one_leg()

% base case: no legs to be inserted
insert_legs(Generated, [], Generated).

% Insert the first leg using insert_one_leg(); and continue.
insert_legs(Generated, [Leg|Legs], Generated2) :-
   insert_one_leg(Generated, Leg, Generated1),
   insert_legs(Generated1, Legs, Generated2).

% head_member(Node, List)
% check whether Node is the head of a member of List.

% base case: node is the head of first item in list.
head_member(Node,[[Node,_]|_]).

% otherwise, keep searching for node in the tail.
head_member(Node,[_|Tail]) :-
  head_member(Node,Tail).

% build_path(Expanded, [[Node,Pred]], Path).

% build_path(Legs, Path)
% Construct a path from a list of legs, by joining the ones that match.

% base case: join the last two legs to form a path of one step.
build_path([[Next,Start],[Start,Start]], [Next,Start]).

% If the first two legs match, add to the front of the path.
build_path([[C,B],[B,A]|Expanded],[C,B,A|Path]) :-
   build_path([[B,A]|Expanded],[B,A|Path]), ! .

% If the above rule fails, we skip the next leg in the list.
build_path([Leg,_SkipLeg|Expanded],Path) :-
   build_path([Leg|Expanded],Path).





% Uniform Cost Search, using Dijkstras Algorithm

% COMP3411/9414/9814 Artificial Intelligence, UNSW, Alan Blair

% solve(Start, Solution, G, N)
% Solution is a path (in reverse order) from start node to a goal state.
% G is the length of the path, N is the number of nodes expanded.

solve(Start, Solution, G, N)  :-
    ucsdijkstra([[Start,Start,0]], [], Solution, G, 1, N).

% ucsdijkstra(Generated, Expanded, Solution, L, N)
%
% The algorithm builds a list of generated "legs" in the form
% Generated = [[Node1,Prev1,G1],[Node2,Prev2,G2],...,[Start,Start,0]]
% The path length G from the start node is stored with each leg,
% and the legs are listed in increasing order of G.
% The expanded nodes are moved to another list (G is discarded)
%  Expanded = [[Node1,Prev1],[Node2,Prev2],...,[Start,Start]]

% If the next leg to be expanded reaches a goal node,
% stop searching, build the path and return it.
ucsdijkstra([[Node,Pred,G]|_Generated], Expanded, Path, G, N, N)  :-
    search_goal(Node),
    build_path([[Node,Pred]|Expanded], Path).

% Extend the leg at the head of the queue by generating the
% successors of its destination node.
% Insert these newly created legs into the list of generated nodes,
% keeping it sorted in increasing order of G; and continue searching.
ucsdijkstra([[Node,Pred,G]| Generated], Expanded, Solution, G1, L, N) :-
    extend(Node, G, Expanded, NewLegs),
    M is L + 1,
    insert_legs(Generated, NewLegs, Generated1),
    ucsdijkstra(Generated1, [[Node,Pred]|Expanded], Solution, G1, M, N).

% Find all successor nodes to this node, and check in each case
% that the new node has not previously been expanded.
extend(Node, G, Expanded, NewLegs) :-
    % write(Node),nl,   % print nodes as they are expanded
    findall([NewNode, Node, G1], (edge_cost(Node, NewNode, C)
    , not(head_member(NewNode, Expanded))
    , G1 is G + C
    ), NewLegs).

% base case: insert leg into an empty list.
insert_one_leg([], Leg, [Leg]).

% If we already knew a shorter path to the same node, discard the new one.
insert_one_leg([Leg1|Generated], Leg, [Leg1|Generated]) :-
    Leg  = [Node,_Pred, G ],
    Leg1 = [Node,_Pred1,G1],
    G >= G1, ! .

% Insert the new leg in its correct place in the list (ordered by G).
insert_one_leg([Leg1|Generated], Leg, [Leg,Leg1|Generated]) :-
    Leg  = [_Node, _Pred, G ],
    Leg1 = [_Node1,_Pred1,G1],
    G < G1, ! .

% Search recursively for the correct place to insert.
insert_one_leg([Leg1|Generated], Leg, [Leg1|Generated1]) :-
    insert_one_leg(Generated, Leg, Generated1).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% infinity cost
infCost(99999).

% set cost
setCost((X,Y), 1):-
  land_or_dropped(X, Y),
  !.

setCost(_, Cost):-
  infCost(Cost).

% edge cost
edge_cost((X,Y), (X1,Y1), Cost) :-
   (
   ( X1 is X, Y1 is Y - 1);
   ( X1 is X, Y1 is Y + 1);
   ( X1 is X - 1, Y1 is Y );
   ( X1 is X + 1, Y1 is Y )),
   setCost((X1,Y1), Cost).

% search goal
search_goal((X,Y)) :-
    agent_at(X,Y).

% shortest Path Distance
shortestPathDistance(Start, Distance):-
  solve(Start, _, Distance, _),
  !.

% shortest path
shortestPath(Start, Path):-
  solve(Start, Path, _, _),
  !.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% find initial intentions
initial_intentions(intents(L,[])):-
   monster(X, Y),
   shortestPath((X, Y), Path),
   include(waterLocation, Path, WaterLocations),
   maplist(init_intention, WaterLocations, L),
   !.

waterLocation((X, Y)):-
  \+land(X, Y).

init_intention((X, Y), [goal(X, Y), []]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% convert percepts to goals
trigger(Stones, Goals):-
  maplist(stone2goal, Stones, Goals).

stone2goal(stone(X,Y), goal(X, Y)).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  updated Intentions of the agent
incorporate_goals(Goals, intents(DropIntentions,PickIntentions), intents(DropIntentions1,PickIntentions1)):-
    include(canPick, Goals, Gs),
    include(canDrop, Goals, Gs1),
    add_all_goals(Gs, PickIntentions, PickIntentions1),
    add_all_goals(Gs1, DropIntentions, DropIntentions1).

% can pick at this location?
canPick(goal(X, Y)):-
    stone_at(X, Y).

% can drop at this location?
canDrop(goal(X, Y)):-
    \+land_or_dropped(X, Y).

% add all goals
add_all_goals([], Intents, Intents).

add_all_goals([Goal | Left], Intents, UpdatedIntent ):-
     add_goal(Goal, Intents, IntentsAdded),
     add_all_goals(Left, IntentsAdded, UpdatedIntent).

% add goal
add_goal(goal(X, Y), Intents,  UpdatedIntent):-
  (goalAppears(goal(X, Y), Intents);  
    shortestPathDistance((X, Y), Distance), infCost(Inf), Distance >= Inf) -> UpdatedIntent = Intents;
  add_goal_rec(goal(X, Y), Intents, UpdatedIntent).

% whether goal appears in the list
goalAppears(Goal, [[Goal, _] | _]):- !.
goalAppears(Goal, [ _ | Left]):- goalAppears(Goal, Left).

% add goal recursively at suitable position
add_goal_rec(goal(X, Y), [],  [ [goal(X, Y), []] ]):- !.

add_goal_rec(goal(X, Y), [ [goal(X1, Y1), Plan] | Left ],  [ [goal(X, Y), []],[goal(X1, Y1), Plan] | Left  ] ):-
    shortestPathDistance((X, Y), Distance),
    shortestPathDistance((X1, Y1), Distance1),
    Distance < Distance1,
    !.

add_goal_rec(Goal, [Head | Left], [Head | Left1]):-
    add_goal_rec(Goal, Left, Left1),
    !.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% computes an action to be taken by the agent as well as the updated Intentions

% If the agent is currently holding a stone,
get_action(intents(DropIntentions,PickIntentions), intents(DropIntentions1,PickIntentions), Action):-
    agent_stones(1),
    take_or_search_action(DropIntentions, DropIntentions1, Action),
    !.

% if the list Int_pick of picking intentions is not empty
get_action(intents(DropIntentions, [Inten | Left]), intents(DropIntentions,PickIntentions1), Action):-
    take_or_search_action([Inten | Left], PickIntentions1, Action),
    !.

% otherwise, no intention is selected  the agent's Intentions should remain as they are
get_action(Intents, Intents, A):-
    agent_at(X, Y),
    A = move(X, Y),
    !.

% Action is applicable
take_or_search_action( [[goal(X, Y), [Action | Left]] | LeftIntens], [[goal(X, Y), Left] | LeftIntens], Action):-
    applicable(Action),
    !.

% otherwise
take_or_search_action( [[goal(X, Y), _] | LeftIntens], [ [goal(X, Y), Plan] | LeftIntens], Action):-
    shortestPath((X, Y), Path),
    search_plan(Path, [_, Action | Plan]),
    !.

% plan from path
search_plan([(X, Y)], [A]):-
    stone_at(X, Y) -> A = pick(X, Y), !;
    A = drop(X, Y), !.

search_plan([(X, Y) | Left], [move(X, Y) | Left1]):-
    search_plan(Left, Left1).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% update the agent's intentions based on observation

% An at(X,Y) observation should not change the agent's intentions.
update_intentions(at(_, _), Intents, Intents):- !.

% In the case of a picked() or dropped() observation, the agent should 
% remove the corresponding plan from its list of intentions
update_intentions(picked(X, Y), intents(DropIntentions,PickIntentions), intents(DropIntentions,PickIntentions1)):-
    deleteFinished((X, Y), PickIntentions, PickIntentions1),!.

update_intentions(dropped(X, Y), intents(DropIntentions,PickIntentions), intents(DropIntentions1,PickIntentions)):-
    deleteFinished((X, Y), DropIntentions, DropIntentions1),!.

% delete finished goals
deleteFinished((X, Y), [[goal(X, Y), _] | Left], Left):- !.

deleteFinished(FinishedCoord, [ HeadGoal | Left], [HeadGoal | LeftDeleted]):-
    deleteFinished(FinishedCoord, Left, LeftDeleted),!.

