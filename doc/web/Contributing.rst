Contributing
============


Kautham Git Development Method
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We are going to follow the GIT cactus model of development (`<https://barro.github.io/2016/02/a-succesful-git-branching-model-considered-harmful/>`_)

The aim of this proposal of development is to have a more linear history of the commits and to avoid scary merge commits.

The main features are:

1) All development happens on the master branch. As a main principle, shared remote branches should be avoided

2) Developers should git rebase their changes regularly so that their local branches would follow the latest origin/master.
    (info regarding git rebase: `<https://git-scm.com/book/en/v2/Git-Branching-Rebasing>`_).

3) Releases are branched out from origin/master


When you have work in a topic branch and have determined that you want to integrate it, you move to that branch and run the rebase command to rebuild the changes on top of your current master branch. If that works well, you can fast-forward your master branch, and you'll end up with a linear project history.

1) Open your topic branch (e.g. branch "experiment", that branched from master and where development of a new feature has been implemented), and make master the new base for the topic branch: ::
    
    $ git checkout experiment
    $ git rebase master

2) Verify that your topic branch complies correctly, i.e. that it works ok with the latest master.

3) Incorporate the features of your topic branch into master. Open master branch and set it at the snapshot that incorporated the changes (i.e. fast-forward your master branch): ::

	$ git checkout master
	$ git merge experiment



Guidelines For Contributing To Kautham
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1) The local branches where we are developing new features for kautham must be continually updated with the latest version of the origin/master branch, i.e. we must periodically rebase our branch with master.

2) Do many commits as recommended below.

3) Update the origin/master with the changes of our branch as soon as possible. Any change regarding the core files of Kautham should be previously informed at kautham@mylist.upc.edu

4) Stable versions of Kautham in origin/master will be tagged, pushed to the github repo, encapsulated as linux packages, and advertised in the Kautham webpage.


Some Recommendations Using Git:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1) Commit Guidelines
	a) Verify that you're not going to commit whitspaces: ::
	
		$ git diff --check

	b) Make each commit a logically separate changeset: split your work into at least one commit per issue, with a useful message per commit (use the staging area).

	c) Create quality commit messages


2) Regarding correcions of commits:

Files modified, or new files created, should be added (staged) before they can be committed: ::

	$ git add modified_file
	$ git add new_file
	$ git commit -m 'One file modified and one file added'

If before commiting we realize that we e.g. wrongly added the new_file, we can undo it: ::

	$ git reset HEAD new_file

If we have already done the commit, it can still be amended, e.g. if we forgot to add a file: ::
	
	$ git commit -m 'initial commit'
	$ git add forgotten_file
	$ git commit --amend

You end up with a single commit and the second commit replaces the results of the first.


3) Cherry-pick: The other way to move introduced work from one branch to another is to cherry-pick it. A cherry-pick in Git is like a rebase for a single commit. From a given branch (e.g. master), apply a change introduced by the commit e43a6 from another branch and create a new commit with this change: ::
	
	$ git cherry-pick e43a6




 
 
 
 
 
 
