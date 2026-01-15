:- module(fp_rules,
    [
        observe_instance/3,
        observation_count/3,
        likelihood_of_surface/3,
        recent_surface/2,
        surface_conflict/1,
        preferred_surface/2,
        search_target/3,
        needs_confirmation/1,
        decision_to_revisit/2
    ]).

:- use_module(library(lists)).

:- dynamic object_observation/4.
% keeping the observations here so we don't lose them between checks

%% observe_instance(+Class,+Surface,+Confidence)
% Just drop a new sighting in the DB with a timestamp.
observe_instance(Class, Surface, Confidence) :-
    get_time(T),
    assertz(object_observation(Class, Surface, Confidence, T)).

%% observation_count(+Class,+Surface,?Count)
% How many times we saw Class on a given Surface (basic counting).
observation_count(Class, Surface, Count) :-
    findall(1, object_observation(Class, Surface, _, _), L),
    length(L, Count).

%% likelihood_of_surface(+Class,+Surface,?Probability)
% Quick add-one smoothing so we don't get 0/0 when nothing is seen yet.
likelihood_of_surface(Class, Surface, Probability) :-
    findall(S, object_observation(Class, S, _, _), AllSurfaces),
    length(AllSurfaces, Total),
    Total > 0,
    observation_count(Class, Surface, Count),
    Probability is (Count + 1) / (Total + 1).

%% recent_surface(+Class,+Surface)
% True if we saw it in the last ~30s (pretty arbitrary but works in sim).
recent_surface(Class, Surface) :-
    get_time(Now),
    object_observation(Class, Surface, _, T),
    Delta is Now - T,
    Delta < 30.

%% surface_conflict(+Class)
% Inference predicate: true if the class has been observed on multiple surfaces.
surface_conflict(Class) :-
    setof(Surface, T^C^object_observation(Class, Surface, C, T), Surfaces),
    length(Surfaces, L),
    L > 1.

%% preferred_surface(+Class,?Surface)
% Pick the surface with the best score; ties handled by list order.
preferred_surface(Class, Surface) :-
    setof(Prob-S, likelihood_of_surface(Class, S, Prob), Ranked),
    last(Ranked, _-Surface).

%% search_target(+Class,?Surface,?Confidence)
% Where to look next plus the confidence we think it has.
search_target(Class, Surface, Confidence) :-
    preferred_surface(Class, Surface),
    likelihood_of_surface(Class, Surface, Confidence).

%% needs_confirmation(+Class)
% If best vs second-best are too close (<=0.2), we should double check.
needs_confirmation(Class) :-
    preferred_surface(Class, SurfaceBest),
    likelihood_of_surface(Class, SurfaceBest, PBest),
    findall(P, (likelihood_of_surface(Class, Surface, P), Surface \= SurfaceBest), Others),
    max_list([0|Others], P2),
    Margin is PBest - P2,
    Margin =< 0.2.

%% decision_to_revisit(+Class,?Surface)
% Suggest going back if the top surface hasn't been seen lately.
decision_to_revisit(Class, Surface) :-
    preferred_surface(Class, Surface),
    \+ recent_surface(Class, Surface).
