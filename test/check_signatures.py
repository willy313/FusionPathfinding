#!/usr/bin/env python3
"""Checks DarkExt.json against the C++ declarations: IDs, parameter types/counts, return types, menus, %n titles."""
import json, re, sys, os
src = os.path.join(os.path.dirname(__file__), '..', 'src')
j = json.load(open(os.path.join(src, 'DarkExt.json'), encoding='utf-8'))['UK English']
cpp = open(os.path.join(src, 'Extension.cpp'), encoding='utf-8').read()
hpp = open(os.path.join(src, 'Extension.hpp'), encoding='utf-8').read()
errors = []

# function name -> (return type, [param types])
decl = {}
for m in re.finditer(r'^\t(void|bool|float|int)\s+(\w+)\(([^)]*)\);', hpp, re.M):
    ret, name, params = m.groups()
    types = []
    for p in [x.strip() for x in params.split(',') if x.strip()]:
        t = re.sub(r'\s*\w+$', '', p).replace(' ', '')   # drop the parameter name
        types.append(t)
    decl[name] = (ret, types)

links = {k: {int(i): n for i, n in re.findall(r'Link%s\((\d+),\s*(\w+)\)' % k, cpp)} for k in ('Action', 'Condition', 'Expression')}
TYPE = {'Integer': 'int', 'Float': 'float', 'Object': 'RunObject*', 'Text': 'constTCHAR*'}

def flat_ids(menu):
    out = []
    for e in menu:
        if isinstance(e[0], int): out.append(e[0])
        else: out += flat_ids(e[1:])
    return out

for kind, key, menukey in (('Action', 'Actions', 'ActionMenu'), ('Condition', 'Conditions', 'ConditionMenu'), ('Expression', 'Expressions', 'ExpressionMenu')):
    items = j[key]
    if sorted(links[kind]) != list(range(len(items))):
        errors.append(f'{kind}: linked IDs {sorted(links[kind])} != 0..{len(items)-1}')
    ids = sorted(flat_ids(j[menukey]))
    if ids != list(range(len(items))):
        errors.append(f'{kind}: menu IDs {ids} do not list every ID exactly once')
    for i, item in enumerate(items):
        fn = links[kind].get(i)
        if fn not in decl: errors.append(f'{kind} {i}: function {fn} not declared in Extension.hpp'); continue
        ret, ptypes = decl[fn]
        jparams = [TYPE[p[0]] for p in item.get('Parameters', [])]
        if jparams != ptypes: errors.append(f'{kind} {i} ({fn}): JSON params {jparams} != C++ params {ptypes}')
        want_ret = {'Action': 'void', 'Condition': 'bool'}.get(kind) or TYPE[item['Returns']]
        if ret != want_ret: errors.append(f'{kind} {i} ({fn}): return {ret}, expected {want_ret}')
        if kind != 'Expression':
            refs = {int(x) for x in re.findall(r'%(\d)', item['Title'])}
            if refs - set(range(len(jparams))): errors.append(f'{kind} {i}: title references missing parameter: {item["Title"]}')
        else:
            if not item['Title'].endswith('('): errors.append(f'Expression {i}: title must end with "("')

# Properties named in C++ must exist in the JSON
titles = {p['Title'] for p in j['Properties']}
for name in re.findall(r'(?:GetPropertyNum|IsPropChecked|GetPropertyStr)\("([^"]+)"sv\)', cpp):
    if name not in titles: errors.append(f'property "{name}" used in C++ is not in DarkExt.json')

if errors:
    print('\n'.join('FAIL: ' + e for e in errors)); sys.exit(1)
print(f'ok: {len(j["Actions"])} actions, {len(j["Conditions"])} conditions, {len(j["Expressions"])} expressions, {len(titles)} properties all consistent')
