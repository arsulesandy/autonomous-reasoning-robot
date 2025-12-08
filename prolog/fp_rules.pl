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

%% observe_instance(+Class,+Surface,+Confidence)
% Assert a new perception event so reasoning can be updated on demand.
observe_instance(Class, Surface, Confidence) :-
    get_time(T),
    assertz(object_observation(Class, Surface, Confidence, T)).

%% observation_count(+Class,+Surface,?Count)
% Inference predicate: how many times we have seen Class on a given Surface.
observation_count(Class, Surface, Count) :-
    findall(1, object_observation(Class, Surface, _, _), L),
    length(L, Count).

%% likelihood_of_surface(+Class,+Surface,?Probability)
% Inference predicate: smoothed likelihood of the class being on the surface.
likelihood_of_surface(Class, Surface, Probability) :-
    findall(S, object_observation(Class, S, _, _), AllSurfaces),
    length(AllSurfaces, Total),
    Total > 0,
    observation_count(Class, Surface, Count),
    Probability is (Count + 1) / (Total + 1).

%% recent_surface(+Class,+Surface)
% Inference predicate: true if we recently saw the class on the surface.
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
% Decision predicate: choose the most likely surface for the class.
preferred_surface(Class, Surface) :-
    setof(Prob-S, likelihood_of_surface(Class, S, Prob), Ranked),
    last(Ranked, _-Surface).

%% search_target(+Class,?Surface,?Confidence)
% Decision predicate: propose a surface to search and return its confidence.
search_target(Class, Surface, Confidence) :-
    preferred_surface(Class, Surface),
    likelihood_of_surface(Class, Surface, Confidence).

%% needs_confirmation(+Class)
% Decision predicate: true if two surfaces are similarly likely and need validation.
needs_confirmation(Class) :-
    preferred_surface(Class, SurfaceBest),
    likelihood_of_surface(Class, SurfaceBest, PBest),
    findall(P, (likelihood_of_surface(Class, Surface, P), Surface \= SurfaceBest), Others),
    max_list([0|Others], P2),
    Margin is PBest - P2,
    Margin =< 0.2.

%% decision_to_revisit(+Class,?Surface)
% Decision predicate: suggest revisiting a surface when we have not seen the class there recently.
decision_to_revisit(Class, Surface) :-
    preferred_surface(Class, Surface),
    \+ recent_surface(Class, Surface).
